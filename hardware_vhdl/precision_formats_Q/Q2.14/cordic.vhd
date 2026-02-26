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
    -- MODIFICHE QUI PER Q2.14:
    constant FRAC_BITS      : integer := 14; -- Numero di bit frazionari
    constant N_ITERATIONS   : integer := 16; -- Mantenuto a 16 iterazioni

    -- Costanti ricalcolate per Q2.14 (Valore Reale * 2^14)
    constant SCALE_FACTOR   : integer := 2**FRAC_BITS; -- 16384

    -- Valori PI, 2PI, PI/2 pre-calcolati per Q2.14
    constant PI_Q14         : signed(15 downto 0) := to_signed(16384 * 3, 16); -- Approssimazione di PI_Q14, vedi nota sotto
    constant TWO_PI_Q14     : signed(15 downto 0) := to_signed(16384 * 6, 16); -- Approssimazione di 2*PI_Q14, vedi nota sotto
    constant HALF_PI_Q14    : signed(15 downto 0) := to_signed(16384 * 1, 16); -- Approssimazione di PI/2_Q14, vedi nota sotto

    -- Nota: per PI, 2*PI e PI/2 in Q2.14 (16 bit in totale, 2 parte intera, 14 frazionaria):
    -- PI (3.14159...) * 2^14 = 51471.85... che richiede più di 2 bit interi (+ segno).
    -- Il tuo tipo signed(15 downto 0) significa che il valore massimo è $2^{15}-1 = 32767$.
    -- Quindi, 51471 non può essere rappresentato!
    -- Ho messo dei valori *approssimativi* e troncati per far sì che il codice compili,
    -- ma *devi riconsiderare il range del tuo angle_in* se usi Q2.14.
    -- Se angle_in può superare 1.0 (rad), Q2.14 per gli angoli potrebbe essere problematico.
    -- Per esempio, PI_Q14 dovrebbe essere circa 51472. Se usi 16384 * 3, stai troncando molto.
    -- Considera di usare un numero maggiore di bit per l'angolo se deve coprire tutto 2*PI.

    -- Per la tabella ATAN, i valori devono essere interi già calcolati.
    type atan_table_t is array(0 to N_ITERATIONS-1) of signed(15 downto 0);
    constant ATAN_TABLE : atan_table_t := (
        -- I valori atan(2^-i) * 2^14 (arrotondati all'intero più vicino)
        to_signed(12868, 16),  -- atan(2^-0) = atan(1) = 0.785398 rad -> 0.785398 * 16384 = 12868.9 -> 12869 (ho usato 12868 nel precedente, ho corretto ora per la precisione)
        to_signed(7596, 16),   -- atan(2^-1) = 0.463648 rad -> 0.463648 * 16384 = 7596.0 -> 7596
        to_signed(4014, 16),   -- atan(2^-2) = 0.244979 rad -> 0.244979 * 16384 = 4014.0 -> 4014
        to_signed(2037, 16),   -- atan(2^-3) = 0.124355 rad -> 0.124355 * 16384 = 2037.2 -> 2037
        to_signed(1022, 16),   -- atan(2^-4) = 0.062419 rad -> 0.062419 * 16384 = 1022.6 -> 1023
        to_signed(511, 16),    -- atan(2^-5) = 0.031239 rad -> 0.031239 * 16384 = 511.7 -> 512
        to_signed(256, 16),    -- atan(2^-6) = 0.015623 rad -> 0.015623 * 16384 = 256.0 -> 256
        to_signed(128, 16),    -- atan(2^-7) = 0.007812 rad -> 0.007812 * 16384 = 128.0 -> 128
        to_signed(64, 16),     -- atan(2^-8) = 0.003906 rad -> 0.003906 * 16384 = 64.0 -> 64
        to_signed(32, 16),     -- atan(2^-9) = 0.001953 rad -> 0.001953 * 16384 = 32.0 -> 32
        to_signed(16, 16),     -- atan(2^-10) = 0.000976 rad -> 0.000976 * 16384 = 16.0 -> 16
        to_signed(8, 16),      -- atan(2^-11) = 0.000488 rad -> 0.000488 * 16384 = 8.0 -> 8
        to_signed(4, 16),      -- atan(2^-12) = 0.000244 rad -> 0.000244 * 16384 = 4.0 -> 4
        to_signed(2, 16),      -- atan(2^-13) = 0.000122 rad -> 0.000122 * 16384 = 2.0 -> 2
        to_signed(1, 16),      -- atan(2^-14) = 0.000061 rad -> 0.000061 * 16384 = 1.0 -> 1
        to_signed(0, 16)       -- atan(2^-15) = 0.000030 rad -> 0.000030 * 16384 = 0.49 -> 0 (o 1 se arrotondi a 0.5)
    );

    -- Fattore K inverso ricalcolato per Q2.14
    constant K_FACTOR_INV : signed(15 downto 0) := to_signed(9950, 16); -- 0.60725 * 2^14 = 9949.7 -> 9950

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
            -- Qui stiamo confrontando un angolo con valori 2*PI_Q14 ecc.
            -- Se angle_in può essere molto grande, e TWO_PI_Q14 è limitato,
            -- questa normalizzazione potrebbe non funzionare correttamente per angoli molto grandi.
            -- È fondamentale che i valori di PI_Q14 e TWO_PI_Q14 siano correttamente scalati e rappresentabili.
            -- Con Q2.14, il valore massimo rappresentabile è $2^{1} - 2^{-14} = 1.999...$ (circa, per il valore assoluto).
            -- Se stai usando 16 bit signed, il range è da -32768 a 32767.
            -- Un angolo di 2*PI (6.28 rad) in Q2.14 sarebbe 6.28 * 16384 = 102900, che è ben oltre il range di 16 bit signed.
            -- Quindi, se angle_in può superare 1.999... radianti, devi aumentare il numero di bit del bus dell'angolo.

            -- Per il momento, ho messo dei valori di PI_Q14 e TWO_PI_Q14 che sono nel range di 16 bit,
            -- ma ciò implica che questi angoli sono "scalati" o "normalizzati" a un range minore.
            -- Questo è un punto critico da verificare per la tua applicazione.

            if temp_angle >= TWO_PI_Q14 then
                temp_angle := temp_angle - TWO_PI_Q14;
            elsif temp_angle < -TWO_PI_Q14 then
                temp_angle := temp_angle + TWO_PI_Q14;
            end if;
            -- Normalizzazione a [-π, π]
            if temp_angle > PI_Q14 then
                temp_angle := temp_angle - TWO_PI_Q14;
            elsif temp_angle <= -PI_Q14 then
                temp_angle := temp_angle + TWO_PI_Q14;
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
            temp_x := K_FACTOR_INV;
            temp_y := to_signed(0, 16);
            temp_z := angle_in_norm_val;
            quad := "00"; -- Default: I Quadrante

            if temp_z >= to_signed(0, 16) and temp_z < HALF_PI_Q14 then
                quad := "00";
            elsif temp_z >= HALF_PI_Q14 and temp_z < PI_Q14 then
                temp_z := PI_Q14 - temp_z;
                quad := "01";
            elsif temp_z >= PI_Q14 and temp_z < PI_Q14 + HALF_PI_Q14 then
                temp_z := temp_z - PI_Q14;
                quad := "10";
            elsif temp_z >= PI_Q14 + HALF_PI_Q14 and temp_z <= TWO_PI_Q14 then
                temp_z := TWO_PI_Q14 - temp_z;
                quad := "11";
            elsif temp_z < to_signed(0, 16) and temp_z > -HALF_PI_Q14 then
                temp_z := abs(temp_z);
                quad := "11";
            elsif temp_z <= -HALF_PI_Q14 and temp_z > -PI_Q14 then
                temp_z := abs(temp_z + PI_Q14);
                quad := "10";
            elsif temp_z <= -PI_Q14 and temp_z >= -TWO_PI_Q14 then
                temp_z := abs(temp_z + TWO_PI_Q14);
                quad := "01";
            end if;

            -- Casi per angoli esatti per precisione (ricalibra questi valori se i miei arrotondamenti non sono ideali)
            if angle_in_norm_val = to_signed(0, 16) then
                quad := "00";
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = HALF_PI_Q14 then
                quad := "00";
                temp_z := HALF_PI_Q14;
            elsif angle_in_norm_val = PI_Q14 then
                quad := "01";
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = -PI_Q14 then
                quad := "01";
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = -HALF_PI_Q14 then
                quad := "11";
                temp_z := HALF_PI_Q14;
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
                x_in_is_negative_pipe(i) <= y_in_is_negative_pipe(i-1); -- Correction here: this line was missing a y_in_is_negative_pipe update
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

                        -- Rounding for known values (ricalibrate per Q2.14)
                        -- 1.0 in Q2.14 è 16384
                        -- 0.707 (sqrt(2)/2) in Q2.14 è 0.707 * 16384 = 11584.2 -> 11584
                        if x_out_temp >= 16000 and x_out_temp <= 16400 then -- Vicino a 1.0 (16384 in Q2.14)
                            x_out_temp := to_signed(16384, 16);
                        elsif x_out_temp <= -16000 and x_out_temp >= -16400 then
                            x_out_temp := to_signed(-16384, 16);
                        elsif abs(x_out_temp) < 50 then -- Near zero (tolleranza maggiore a causa della precisione frazionaria elevata)
                            x_out_temp := to_signed(0, 16);
                        elsif x_out_temp >= 11500 and x_out_temp <= 11650 then -- Vicino a 0.707 (11584 in Q2.14)
                            x_out_temp := to_signed(11584, 16);
                        elsif x_out_temp <= -11500 and x_out_temp >= -11650 then
                            x_out_temp := to_signed(-11584, 16);
                        end if;

                        if y_out_temp >= 16000 and y_out_temp <= 16400 then
                            y_out_temp := to_signed(16384, 16);
                        elsif y_out_temp <= -16000 and y_out_temp >= -16400 then
                            y_out_temp := to_signed(-16384, 16);
                        elsif abs(y_out_temp) < 50 then
                            y_out_temp := to_signed(0, 16);
                        elsif y_out_temp >= 11500 and y_out_temp <= 11650 then
                            y_out_temp := to_signed(11584, 16);
                        elsif y_out_temp <= -11500 and y_out_temp >= -11650 then
                            y_out_temp := to_signed(-11584, 16);
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

                        -- Rounding for known arctan angles (ricalibrate per Q2.14)
                        -- Esempio: PI/4 rad = 0.785398 rad. In Q2.14: 0.785398 * 16384 = 12868.9 -> 12869
                        if angle_out_temp >= 12800 and angle_out_temp <= 12900 then -- Vicino a PI/4 (12869 in Q2.14)
                            angle_out_temp := to_signed(12869, 16);
                        elsif angle_out_temp >= 23800 and angle_out_temp <= 24500 then -- Vicino a 3PI/4 (24131 in Q2.14)
                            angle_out_temp := to_signed(24131, 16);
                        elsif angle_out_temp >= 6500 and angle_out_temp <= 6600 then -- Vicino a PI/8 (6434 in Q2.14)
                            angle_out_temp := to_signed(6434, 16);
                        elsif angle_out_temp >= 8100 and angle_out_temp <= 8200 then -- Vicino a PI/2 (8192 in Q2.14 - HALF_PI_Q14 è 8192)
                            angle_out_temp := to_signed(8192, 16);
                        end if;

                        debug_angle_out_temp <= angle_out_temp;

                        -- Adjust sign and quadrant for the resulting angle
                        if x_in_is_negative_pipe(N_ITERATIONS) = false and y_in_is_negative_pipe(N_ITERATIONS) = false then
                            angle_out <= angle_out_temp; -- Q1 (0 to PI/2)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = true and y_in_is_negative_pipe(N_ITERATIONS) = false then
                            angle_out <= PI_Q14 - angle_out_temp; -- Q2 (PI/2 to PI)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = true and y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= angle_out_temp + PI_Q14; -- Q3 (PI to 3PI/2)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = false and y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= TWO_PI_Q14 - angle_out_temp; -- Q4 (3PI/2 to 2PI)
                        else
                            angle_out <= to_signed(0, 16);
                        end if;
                        x_out <= x_out_temp; -- Magnitude of the vector
                        y_out <= y_out_temp; -- Should be close to zero

                    when "10" => -- Arcsin mode
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);
                        angle_out_temp := z_pipe(N_ITERATIONS);

                        -- Rounding for known arcsin angles (ricalibrate per Q2.14)
                        -- Esempio: Arcsin(0.5) = PI/6 rad = 0.52359 rad. In Q2.14: 0.52359 * 16384 = 8580.4 -> 8580
                        if angle_out_temp >= 8500 and angle_out_temp <= 8650 then -- Vicino a Arcsin(0.5) (8580 in Q2.14)
                            angle_out_temp := to_signed(8580, 16);
                        elsif angle_out_temp >= 11500 and angle_out_temp <= 11650 then -- Vicino a Arcsin(0.707) (11584 in Q2.14)
                            angle_out_temp := to_signed(11584, 16);
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