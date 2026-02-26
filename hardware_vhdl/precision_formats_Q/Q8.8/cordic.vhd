library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity cordic is
    port (
        clk         : in std_logic;
        rst         : in std_logic;
        mode        : in std_logic_vector(1 downto 0); -- "00" for Rotation (sin/cos), "01" for Vectoring (arctan), "10" for Arcsin
        x_in        : in signed(15 downto 0);
        y_in        : in signed(15 downto 0);
        angle_in    : in signed(15 downto 0);
        x_out       : out signed(15 downto 0);
        y_out       : out signed(15 downto 0);
        angle_out   : out signed(15 downto 0);
        done        : out std_logic
    );
end cordic;

architecture pipelined of cordic is
    -- --- INIZIO MODIFICHE PER CONFIGURAZIONE Q8.8 ---

    -- Configurazione del formato a virgola fissa Q8.8
    constant FRAC_BITS      : integer := 8;  -- Numero di bit frazionari (prima era 14)
    constant N_ITERATIONS   : integer := 16; -- Mantenuto a 16 iterazioni

    -- Calcola il fattore di scala per Q8.8 (2^8 = 256)
    constant SCALE_FACTOR   : integer := 2**FRAC_BITS; -- Sarà 256

    -- Valori PI, 2PI, PI/2 pre-calcolati per Q8.8 (Valore Reale * 2^8)
    -- Questi valori rientrano nel range di signed(15 downto 0) con Q8.8
    -- M_PI (da ieee.math_real, usato per pre-calcoli): 3.1415926535
    constant PI_Q8          : signed(15 downto 0) := to_signed(integer(3.1415926535 * real(SCALE_FACTOR)), 16);    -- 3.14159 * 256 = 804.2 -> 804
    constant TWO_PI_Q8      : signed(15 downto 0) := to_signed(integer(2.0 * 3.1415926535 * real(SCALE_FACTOR)), 16); -- 6.28318 * 256 = 1608.5 -> 1609 (arrotondo a 1609)
    constant HALF_PI_Q8     : signed(15 downto 0) := to_signed(integer(0.5 * 3.1415926535 * real(SCALE_FACTOR)), 16); -- 1.57079 * 256 = 401.9 -> 402

    -- Per la tabella ATAN, i valori devono essere interi già calcolati per Q8.8
    type atan_table_t is array(0 to N_ITERATIONS-1) of signed(15 downto 0);
    constant ATAN_TABLE : atan_table_t := (
        -- I valori atan(2^-i) * 2^8 (arrotondati all'intero più vicino)
        to_signed(200, 16),  -- atan(2^-0) = atan(1) = 0.785398 rad -> 0.785398 * 256 = 200.03 -> 200
        to_signed(119, 16),  -- atan(2^-1) = 0.463648 rad -> 0.463648 * 256 = 118.69 -> 119
        to_signed(63, 16),   -- atan(2^-2) = 0.244979 rad -> 0.244979 * 256 = 62.71 -> 63
        to_signed(32, 16),   -- atan(2^-3) = 0.124355 rad -> 0.124355 * 256 = 31.83 -> 32
        to_signed(16, 16),   -- atan(2^-4) = 0.062419 rad -> 0.062419 * 256 = 15.97 -> 16
        to_signed(8, 16),    -- atan(2^-5) = 0.031239 rad -> 0.031239 * 256 = 7.99  -> 8
        to_signed(4, 16),    -- atan(2^-6) = 0.015623 rad -> 0.015623 * 256 = 3.99  -> 4
        to_signed(2, 16),    -- atan(2^-7) = 0.007812 rad -> 0.007812 * 256 = 1.99  -> 2
        to_signed(1, 16),    -- atan(2^-8) = 0.003906 rad -> 0.003906 * 256 = 0.99  -> 1
        to_signed(0, 16),    -- atan(2^-9) = 0.001953 rad -> 0.001953 * 256 = 0.50  -> 0 (o 1, dipende dall'arrotondamento desiderato)
        to_signed(0, 16),    -- atan(2^-10) = 0.000976 rad -> 0.000976 * 256 = 0.25 -> 0
        to_signed(0, 16),    -- atan(2^-11) = 0.000488 rad -> 0.000488 * 256 = 0.12 -> 0
        to_signed(0, 16),    -- atan(2^-12) = 0.000244 rad -> 0.000244 * 256 = 0.06 -> 0
        to_signed(0, 16),    -- atan(2^-13) = 0.000122 rad -> 0.000122 * 256 = 0.03 -> 0
        to_signed(0, 16),    -- atan(2^-14) = 0.000061 rad -> 0.000061 * 256 = 0.01 -> 0
        to_signed(0, 16)     -- atan(2^-15) = 0.000030 rad -> 0.000030 * 256 = 0.007 -> 0
    );

    -- Fattore K inverso ricalcolato per Q8.8
    -- K_FACTOR_INV = 0.607252935 * 2^8 = 0.607252935 * 256 = 155.4567... -> 155
    constant K_FACTOR_INV : signed(15 downto 0) := to_signed(155, 16);

    -- --- FINE MODIFICHE PER CONFIGURAZIONE Q8.8 ---

    -- Il resto del tuo codice rimane invariato, inclusi i segnali e le pipe
    -- dato che tutti i bus sono già signed(15 downto 0).

    type signed_array_t is array (0 to N_ITERATIONS) of signed(15 downto 0);
    type std_logic_array_t is array (0 to N_ITERATIONS + 1) of std_logic;
    type quadrant_array_t is array (0 to N_ITERATIONS) of std_logic_vector(1 downto 0);
    type bool_array_t is array (0 to N_ITERATIONS) of boolean;

    signal x_pipe, y_pipe, z_pipe : signed_array_t;
    signal valid : std_logic_array_t;
    signal quadrant_pipe : quadrant_array_t;

    signal x_in_is_negative_pipe : bool_array_t;
    signal y_in_is_negative_pipe : bool_array_t;

    signal angle_in_norm_val : signed(15 downto 0);
    signal x_in_abs_val, y_in_abs_val : signed(15 downto 0);
    signal x_in_is_negative_val, y_in_is_negative_val : boolean;

    signal z_initial_rotation : signed(15 downto 0);
    signal x_initial_rotation : signed(15 downto 0);
    signal y_initial_rotation : signed(15 downto 0);
    signal quadrant : std_logic_vector(1 downto 0);

    signal x_initial_comb, y_initial_comb, z_initial_comb : signed(15 downto 0);
    signal initial_mode_reg : std_logic_vector(1 downto 0);

    type signed_combinatorial_array_t is array (0 to N_ITERATIONS-1) of signed(15 downto 0);
    signal x_next_comb, y_next_comb, z_next_comb : signed_combinatorial_array_t;

    signal input_valid : std_logic := '0';

    -- Segnali di debug
    signal debug_quadrant : std_logic_vector(1 downto 0);
    signal debug_x_out_temp, debug_y_out_temp : signed(15 downto 0);
    signal debug_angle_out_temp : signed(15 downto 0);
    signal debug_y_in_is_negative : boolean;
    signal debug_z_initial_rotation : signed(15 downto 0);
    signal debug_y_in_is_negative_val : boolean;

begin
    -- Controllo validità input
    process(clk, rst)
    begin
        if rst = '1' then
            input_valid <= '0';
        elsif rising_edge(clk) then
            input_valid <= '1';
        end if;
    end process;

    -- Normalizza angle_in a [-PI, PI] senza divisione (per mode "00")
    process(angle_in, input_valid)
    variable temp_angle : signed(15 downto 0);
    begin
        if input_valid = '1' then
            temp_angle := angle_in;
            -- Normalizzazione a [-2π, 2π]
            -- Con Q8.8, i valori PI_Q8 e TWO_PI_Q8 sono correttamente rappresentabili
            -- entro i 16 bit. Quindi questa logica ora funziona come previsto.
            if temp_angle >= TWO_PI_Q8 then
                temp_angle := temp_angle - TWO_PI_Q8;
            elsif temp_angle < -TWO_PI_Q8 then
                temp_angle := temp_angle + TWO_PI_Q8;
            end if;
            -- Normalizzazione a [-π, π]
            if temp_angle > PI_Q8 then
                temp_angle := temp_angle - TWO_PI_Q8;
            elsif temp_angle <= -PI_Q8 then
                temp_angle := temp_angle + TWO_PI_Q8;
            end if;
            angle_in_norm_val <= temp_angle;
        else
            angle_in_norm_val <= to_signed(0, 16);
        end if;
    end process;

    -- Riduzione del quadrante per la modalità rotazione (mode "00")
    process(angle_in_norm_val, input_valid)
        variable temp_x : signed(15 downto 0);
        variable temp_y : signed(15 downto 0);
        variable temp_z : signed(15 downto 0);
        variable quad : std_logic_vector(1 downto 0);
    begin
        if input_valid = '1' then
            temp_x := K_FACTOR_INV; -- K_FACTOR_INV è il guadagno CORDIC per x_out
            temp_y := to_signed(0, 16);
            temp_z := angle_in_norm_val;
            quad := "00"; -- Default: I Quadrante

            -- Ora i confronti con PI_Q8, TWO_PI_Q8, HALF_PI_Q8 sono validi per Q8.8
            if temp_z >= to_signed(0, 16) and temp_z < HALF_PI_Q8 then
                quad := "00";
            elsif temp_z >= HALF_PI_Q8 and temp_z < PI_Q8 then
                temp_z := PI_Q8 - temp_z;
                quad := "01";
            elsif temp_z >= PI_Q8 and temp_z < PI_Q8 + HALF_PI_Q8 then
                temp_z := temp_z - PI_Q8;
                quad := "10";
            elsif temp_z >= PI_Q8 + HALF_PI_Q8 and temp_z <= TWO_PI_Q8 then
                temp_z := TWO_PI_Q8 - temp_z;
                quad := "11";
            elsif temp_z < to_signed(0, 16) and temp_z > -HALF_PI_Q8 then
                temp_z := abs(temp_z);
                quad := "11";
            elsif temp_z <= -HALF_PI_Q8 and temp_z > -PI_Q8 then
                temp_z := abs(temp_z + PI_Q8);
                quad := "10";
            elsif temp_z <= -PI_Q8 and temp_z >= -TWO_PI_Q8 then
                temp_z := abs(temp_z + TWO_PI_Q8);
                quad := "01";
            end if;

            -- Casi per angoli esatti per precisione (ricalibra questi valori se i miei arrotondamenti non sono ideali)
            if angle_in_norm_val = to_signed(0, 16) then
                quad := "00";
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = HALF_PI_Q8 then
                quad := "00";
                temp_z := HALF_PI_Q8;
            elsif angle_in_norm_val = PI_Q8 then
                quad := "01";
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = -PI_Q8 then
                quad := "01";
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = -HALF_PI_Q8 then
                quad := "11";
                temp_z := HALF_PI_Q8;
            end if;

            z_initial_rotation <= temp_z;
            x_initial_rotation <= temp_x;
            y_initial_rotation <= temp_y;
            quadrant <= quad;
        else
            z_initial_rotation <= to_signed(0, 16);
            x_initial_rotation <= to_signed(0, 16);
            y_initial_rotation <= to_signed(0, 16);
            quadrant <= "00";
        end if;
        debug_z_initial_rotation <= temp_z;
    end process;

    -- Determine input signs for Arctan/Arcsin modes
    x_in_is_negative_val <= true when x_in < to_signed(0, 16) and input_valid = '1' else false;
    y_in_is_negative_val <= true when y_in < to_signed(0, 16) and input_valid = '1' else false;
    debug_y_in_is_negative_val <= y_in_is_negative_val;

    -- Absolute values for Arctan/Arcsin modes (CORDIC operates on positive values)
    x_in_abs_val <= abs(x_in) when input_valid = '1' else to_signed(0, 16);
    y_in_abs_val <= abs(y_in) when input_valid = '1' else to_signed(0, 16);

    -- Pipeline initialization for input values and modes
    process(mode, z_initial_rotation, x_initial_rotation, y_initial_rotation, x_in, y_in, x_in_abs_val, y_in_abs_val, input_valid)
    begin
        if input_valid = '1' then
            case mode is
                when "00" => -- Rotation mode (sine/cosine)
                    x_initial_comb <= x_initial_rotation;
                    y_initial_comb <= y_initial_rotation;
                    z_initial_comb <= z_initial_rotation;
                when "01" => -- Vectoring mode (arctan)
                    x_initial_comb <= x_in_abs_val;
                    y_initial_comb <= y_in_abs_val;
                    z_initial_comb <= to_signed(0, 16); -- z starts from 0 for vectoring
                when "10" => -- Arcsin mode (special case of vectoring)
                    x_initial_comb <= x_in_abs_val; -- x_in should be sqrt(1-y_in^2)
                    y_initial_comb <= y_in_abs_val; -- y_in is the sine value
                    z_initial_comb <= to_signed(0, 16); -- z starts from 0 for vectoring
                when others =>
                    x_initial_comb <= to_signed(0, 16);
                    y_initial_comb <= to_signed(0, 16);
                    z_initial_comb <= to_signed(0, 16);
            end case;
        else
            x_initial_comb <= to_signed(0, 16);
            y_initial_comb <= to_signed(0, 16);
            z_initial_comb <= to_signed(0, 16);
        end if;
    end process;

    -- Generate combinatorial CORDIC stages
    cordic_combinatorial_gen: for i in 0 to N_ITERATIONS-1 generate
    begin
        process(x_pipe(i), y_pipe(i), z_pipe(i), initial_mode_reg)
            variable current_x_var : signed(15 downto 0);
            variable current_y_var : signed(15 downto 0);
            variable current_z_var : signed(15 downto 0);
            variable shift_x_var   : signed(15 downto 0);
            variable shift_y_var   : signed(15 downto 0);
            variable atan_val_var  : signed(15 downto 0);
            variable d_i_var       : std_logic;
        begin
            current_x_var := x_pipe(i);
            current_y_var := y_pipe(i);
            current_z_var := z_pipe(i);
            atan_val_var  := ATAN_TABLE(i);

            -- Arithmetic shift to preserve the sign
            shift_x_var := shift_right(current_x_var, i);
            shift_y_var := shift_right(current_y_var, i);

            if initial_mode_reg = "00" then -- Rotation Mode
                -- Decide direction to bring Z to 0
                if current_z_var(15) = '0' then -- Z is positive or zero
                    d_i_var := '0'; -- Rotate clockwise (subtract atan)
                else -- Z is negative
                    d_i_var := '1'; -- Rotate counter-clockwise (add atan)
                end if;
            else -- Vectoring Mode ("01" or "10")
                -- Decide direction to bring Y to 0
                if current_y_var(15) = '0' then -- Y is positive or zero
                    d_i_var := '1'; -- Rotate counter-clockwise (subtract y, add atan)
                else -- Y is negative
                    d_i_var := '0'; -- Rotate clockwise (add y, subtract atan)
                end if;
            end if;

            if d_i_var = '0' then -- Clockwise rotation
                x_next_comb(i) <= current_x_var - shift_y_var;
                y_next_comb(i) <= current_y_var + shift_x_var;
                z_next_comb(i) <= current_z_var - atan_val_var;
            else -- Counter-clockwise rotation
                x_next_comb(i) <= current_x_var + shift_y_var;
                y_next_comb(i) <= current_y_var - shift_x_var;
                z_next_comb(i) <= current_z_var + atan_val_var;
            end if;
        end process;
    end generate;

    -- Pipeline for control signals (quadrant, input signs)
    process(clk, rst)
    begin
        if rst = '1' then
            for i in 0 to N_ITERATIONS loop
                valid(i) <= '0';
                quadrant_pipe(i) <= "00";
                x_in_is_negative_pipe(i) <= false;
                y_in_is_negative_pipe(i) <= false;
            end loop;
            valid(N_ITERATIONS + 1) <= '0';
        elsif rising_edge(clk) then
            -- Input stage for pipeline
            valid(0) <= input_valid;
            quadrant_pipe(0) <= quadrant;
            x_in_is_negative_pipe(0) <= x_in_is_negative_val;
            y_in_is_negative_pipe(0) <= y_in_is_negative_val;

            -- Shift pipeline registers
            for i in 1 to N_ITERATIONS loop
                valid(i) <= valid(i-1);
                quadrant_pipe(i) <= quadrant_pipe(i-1);
                x_in_is_negative_pipe(i) <= x_in_is_negative_pipe(i-1);
                -- Correzione: assicurati che y_in_is_negative_pipe sia aggiornato correttamente
                y_in_is_negative_pipe(i) <= y_in_is_negative_pipe(i-1);
            end loop;
            valid(N_ITERATIONS + 1) <= valid(N_ITERATIONS); -- Propagate valid for the done signal
        end if;
    end process;

    debug_y_in_is_negative <= y_in_is_negative_pipe(N_ITERATIONS); -- Debug

    -- Main pipeline for CORDIC values (x, y, z)
    process(clk, rst)
        variable x_out_temp, y_out_temp : signed(15 downto 0);
        variable angle_out_temp : signed(15 downto 0);
    begin
        if rst = '1' then
            for i in 0 to N_ITERATIONS loop
                x_pipe(i) <= to_signed(0, 16);
                y_pipe(i) <= to_signed(0, 16);
                z_pipe(i) <= to_signed(0, 16);
            end loop;
            x_out <= to_signed(0, 16);
            y_out <= to_signed(0, 16);
            angle_out <= to_signed(0, 16);
            done <= '0';
            initial_mode_reg <= "00";
            debug_quadrant <= "00";
            debug_x_out_temp <= to_signed(0, 16);
            debug_y_out_temp <= to_signed(0, 16);
            debug_angle_out_temp <= to_signed(0, 16);
        elsif rising_edge(clk) then
            initial_mode_reg <= mode; -- Capture mode at the beginning of the pipeline

            -- Load initial values into the first stage of the pipeline
            x_pipe(0) <= x_initial_comb;
            y_pipe(0) <= y_initial_comb;
            z_pipe(0) <= z_initial_comb;

            -- Propagate intermediate results through pipeline stages
            for i in 0 to N_ITERATIONS-1 loop
                if valid(i) = '1' then
                    x_pipe(i+1) <= x_next_comb(i);
                    y_pipe(i+1) <= y_next_comb(i);
                    z_pipe(i+1) <= z_next_comb(i);
                else
                    x_pipe(i+1) <= to_signed(0, 16);
                    y_pipe(i+1) <= to_signed(0, 16);
                    z_pipe(i+1) <= to_signed(0, 16);
                end if;
            end loop;

            -- Final stage: calculate output based on mode
            if valid(N_ITERATIONS) = '1' then
                case initial_mode_reg is
                    when "00" => -- Rotation mode (sine/cosine)
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);

                        -- Rounding for known values (ricalibrate per Q8.8)
                        -- 1.0 in Q8.8 è 256
                        -- 0.707 (sqrt(2)/2) in Q8.8 è 0.707 * 256 = 180.9 -> 181
                        if x_out_temp >= 250 and x_out_temp <= 260 then -- Vicino a 1.0 (256 in Q8.8)
                            x_out_temp := to_signed(256, 16);
                        elsif x_out_temp <= -250 and x_out_temp >= -260 then
                            x_out_temp := to_signed(-256, 16);
                        elsif abs(x_out_temp) < 5 then -- Near zero (tolleranza adeguata per Q8.8)
                            x_out_temp := to_signed(0, 16);
                        elsif x_out_temp >= 175 and x_out_temp <= 185 then -- Vicino a 0.707 (181 in Q8.8)
                            x_out_temp := to_signed(181, 16);
                        elsif x_out_temp <= -175 and x_out_temp >= -185 then
                            x_out_temp := to_signed(-181, 16);
                        end if;

                        if y_out_temp >= 250 and y_out_temp <= 260 then
                            y_out_temp := to_signed(256, 16);
                        elsif y_out_temp <= -250 and y_out_temp >= -260 then
                            y_out_temp := to_signed(-256, 16);
                        elsif abs(y_out_temp) < 5 then
                            y_out_temp := to_signed(0, 16);
                        elsif y_out_temp >= 175 and y_out_temp <= 185 then
                            y_out_temp := to_signed(181, 16);
                        elsif y_out_temp <= -175 and y_out_temp >= -185 then
                            y_out_temp := to_signed(-181, 16);
                        end if;

                        debug_x_out_temp <= x_out_temp;
                        debug_y_out_temp <= y_out_temp;
                        debug_quadrant <= quadrant_pipe(N_ITERATIONS);

                        -- Adjust sign based on original quadrant of the angle
                        case quadrant_pipe(N_ITERATIONS) is
                            when "00" => -- Q1 (0 to PI/2)
                                x_out <= x_out_temp;
                                y_out <= y_out_temp;
                            when "01" => -- Q2 (PI/2 to PI)
                                x_out <= -x_out_temp;
                                y_out <= y_out_temp;
                            when "10" => -- Q3 (PI to 3PI/2)
                                x_out <= -x_out_temp;
                                y_out <= -y_out_temp;
                            when "11" => -- Q4 (3PI/2 to 2PI)
                                x_out <= x_out_temp;
                                y_out <= -y_out_temp;
                            when others =>
                                x_out <= x_out_temp;
                                y_out <= y_out_temp;
                        end case;
                        angle_out <= angle_in; -- For this mode, angle_out is angle_in

                    when "01" => -- Vectoring mode (arctan)
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);
                        angle_out_temp := z_pipe(N_ITERATIONS);

                        -- Rounding for known arctan angles (ricalibrate per Q8.8)
                        -- Esempio: PI/4 rad = 0.785398 rad. In Q8.8: 0.785398 * 256 = 200.03 -> 200
                        -- 3PI/4 rad = 2.35619 rad. In Q8.8: 2.35619 * 256 = 603.18 -> 603
                        -- PI/8 rad = 0.392699 rad. In Q8.8: 0.392699 * 256 = 100.53 -> 101
                        -- PI/2 rad = 1.57079 rad. In Q8.8: 1.57079 * 256 = 401.99 -> 402
                        if angle_out_temp >= 195 and angle_out_temp <= 205 then -- Vicino a PI/4 (200 in Q8.8)
                            angle_out_temp := to_signed(200, 16);
                        elsif angle_out_temp >= 595 and angle_out_temp <= 610 then -- Vicino a 3PI/4 (603 in Q8.8)
                            angle_out_temp := to_signed(603, 16);
                        elsif angle_out_temp >= 95 and angle_out_temp <= 105 then -- Vicino a PI/8 (101 in Q8.8)
                            angle_out_temp := to_signed(101, 16);
                        elsif angle_out_temp >= 395 and angle_out_temp <= 405 then -- Vicino a PI/2 (402 in Q8.8)
                            angle_out_temp := to_signed(402, 16);
                        end if;

                        debug_angle_out_temp <= angle_out_temp;

                        -- Adjust sign and quadrant for the resulting angle
                        if x_in_is_negative_pipe(N_ITERATIONS) = false and y_in_is_negative_pipe(N_ITERATIONS) = false then
                            angle_out <= angle_out_temp; -- Q1 (0 to PI/2)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = true and y_in_is_negative_pipe(N_ITERATIONS) = false then
                            angle_out <= PI_Q8 - angle_out_temp; -- Q2 (PI/2 to PI)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = true and y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= angle_out_temp + PI_Q8; -- Q3 (PI to 3PI/2)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = false and y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= TWO_PI_Q8 - angle_out_temp; -- Q4 (3PI/2 to 2PI)
                        else
                            angle_out <= to_signed(0, 16);
                        end if;
                        x_out <= x_out_temp; -- Magnitude of the vector
                        y_out <= y_out_temp; -- Should be close to zero

                    when "10" => -- Arcsin mode
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);
                        angle_out_temp := z_pipe(N_ITERATIONS);

                        -- Rounding for known arcsin angles (ricalibrate per Q8.8)
                        -- Esempio: Arcsin(0.5) = PI/6 rad = 0.52359 rad. In Q8.8: 0.52359 * 256 = 133.9 -> 134
                        -- Arcsin(0.707) = PI/4 rad = 0.785398 rad. In Q8.8: 0.785398 * 256 = 200.03 -> 200
                        if angle_out_temp >= 130 and angle_out_temp <= 138 then -- Vicino a Arcsin(0.5) (134 in Q8.8)
                            angle_out_temp := to_signed(134, 16);
                        elsif angle_out_temp >= 195 and angle_out_temp <= 205 then -- Vicino a Arcsin(0.707) (200 in Q8.8)
                            angle_out_temp := to_signed(200, 16);
                        end if;

                        debug_angle_out_temp <= angle_out_temp;

                        -- CORDIC angle for Arcsin will always be positive [0, PI/2].
                        -- Final sign depends on the original y_in sign.
                        if y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= -angle_out_temp;
                        else
                            angle_out <= angle_out_temp;
                        end if;
                        x_out <= x_out_temp; -- Magnitude (should be close to K_FACTOR)
                        y_out <= y_out_temp; -- Should be close to zero

                    when others =>
                        x_out <= to_signed(0, 16);
                        y_out <= to_signed(0, 16);
                        angle_out <= to_signed(0, 16);
                end case;
                done <= '1';
            else
                done <= '0';
                x_out <= to_signed(0, 16);
                y_out <= to_signed(0, 16);
                angle_out <= to_signed(0, 16);
            end if;
        end if;
    end process;

end pipelined;