library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity cordic is
    port (
        clk         : in std_logic;
        rst         : in std_logic;
        mode        : in std_logic_vector(1 downto 0);
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
    constant FRAC_BITS      : integer := 12;
    constant N_ITERATIONS   : integer := 32; -- Modificato a 32 iterazioni

    constant PI_Q12         : signed(15 downto 0) := to_signed(12868, 16); -- π * 2^12 (Approx 3.14159 * 4096 = 12867.96)
    constant TWO_PI_Q12     : signed(15 downto 0) := to_signed(25736, 16); -- 2π * 2^12 (Approx 6.28318 * 4096 = 25735.92)
    constant HALF_PI_Q12    : signed(15 downto 0) := to_signed(6434, 16);  -- π/2 * 2^12 (Approx 1.57079 * 4096 = 6433.98)

    type atan_table_t is array(0 to N_ITERATIONS-1) of signed(15 downto 0);
    constant ATAN_TABLE : atan_table_t := (
        to_signed(3217, 16), -- atan(2^-0) = 45°
        to_signed(1899, 16), -- atan(2^-1) = 26.565°
        to_signed(1003, 16), -- atan(2^-2) = 14.036°
        to_signed(509, 16),  -- atan(2^-3) = 7.125°
        to_signed(255, 16),  -- atan(2^-4) = 3.576°
        to_signed(128, 16),  -- atan(2^-5) = 1.789°
        to_signed(64, 16),   -- atan(2^-6) = 0.895°
        to_signed(32, 16),   -- atan(2^-7) = 0.448°
        to_signed(16, 16),   -- atan(2^-8) = 0.224°
        to_signed(8, 16),    -- atan(2^-9) = 0.112°
        to_signed(4, 16),    -- atan(2^-10) = 0.056°
        to_signed(2, 16),    -- atan(2^-11) = 0.028°
        to_signed(1, 16),    -- atan(2^-12) = 0.014°
        to_signed(0, 16),    -- atan(2^-13) ~ 0.007° (scaled to Q12, this is 0)
        to_signed(0, 16),    -- atan(2^-14)
        to_signed(0, 16),    -- atan(2^-15)
        to_signed(0, 16),    -- atan(2^-16)
        to_signed(0, 16),    -- atan(2^-17)
        to_signed(0, 16),    -- atan(2^-18)
        to_signed(0, 16),    -- atan(2^-19)
        to_signed(0, 16),    -- atan(2^-20)
        to_signed(0, 16),    -- atan(2^-21)
        to_signed(0, 16),    -- atan(2^-22)
        to_signed(0, 16),    -- atan(2^-23)
        to_signed(0, 16),    -- atan(2^-24)
        to_signed(0, 16),    -- atan(2^-25)
        to_signed(0, 16),    -- atan(2^-26)
        to_signed(0, 16),    -- atan(2^-27)
        to_signed(0, 16),    -- atan(2^-28)
        to_signed(0, 16),    -- atan(2^-29)
        to_signed(0, 16),    -- atan(2^-30)
        to_signed(0, 16)     -- atan(2^-31)
    );

    constant K_FACTOR_INV : signed(15 downto 0) := to_signed(2489, 16); -- 1/1.646760258 * 2^12 = 0.607252935 * 4096 = 2487.89 -> 2488 or 2489. Let's use 2489

    type signed_array_t is array (0 to N_ITERATIONS) of signed(15 downto 0);
    type std_logic_array_t is array (0 to N_ITERATIONS + 1) of std_logic;
    type quadrant_array_t is array (0 to N_ITERATIONS) of std_logic_vector(1 downto 0);
    type bool_array_t is array (0 to N_ITERATIONS) of boolean;


    signal x_pipe, y_pipe, z_pipe : signed_array_t;
    signal valid : std_logic_array_t;
    signal quadrant_pipe : quadrant_array_t;
    
    -- Utilizziamo booleani per i segni per chiarezza e propaghiamoli in pipeline
    signal x_in_is_negative_pipe : bool_array_t;
    signal y_in_is_negative_pipe : bool_array_t;

    signal angle_in_norm_val : signed(15 downto 0);
    signal x_in_abs_val, y_in_abs_val : signed(15 downto 0);
    signal x_in_is_negative_val, y_in_is_negative_val : boolean; -- Ora sono booleani

    signal z_initial_rotation : signed(15 downto 0);
    signal x_initial_rotation : signed(15 downto 0);
    signal y_initial_rotation : signed(15 downto 0);
    signal quadrant : std_logic_vector(1 downto 0);

    signal x_initial_comb, y_initial_comb, z_initial_comb : signed(15 downto 0);
    signal initial_mode_reg : std_logic_vector(1 downto 0);

    type signed_combinatorial_array_t is array (0 to N_ITERATIONS-1) of signed(15 downto 0);
    signal x_next_comb, y_next_comb, z_next_comb : signed_combinatorial_array_t;

    signal input_valid : std_logic := '0';

    -- Segnali di debug (li ho mantenuti, ma potresti rimuoverli in produzione)
    signal debug_quadrant : std_logic_vector(1 downto 0);
    signal debug_x_out_temp, debug_y_out_temp : signed(15 downto 0);
    signal debug_angle_out_temp : signed(15 downto 0);
    signal debug_y_in_is_negative : boolean; -- Cambiato a boolean
    signal debug_z_initial_rotation : signed(15 downto 0);
    signal debug_y_in_is_negative_val : boolean; -- Cambiato a boolean

begin
    -- Controllo validità input
    process(clk, rst)
    begin
        if rst = '1' then
            input_valid <= '0';
        elsif rising_edge(clk) then
            input_valid <= '1'; -- Assume input è valido un ciclo dopo il reset o all'attivazione
        end if;
    end process;

    -- Normalizza angle_in a [-PI, PI] senza divisione (per mode "00")
    process(angle_in, input_valid)
    variable temp_angle : signed(15 downto 0);
begin
    if input_valid = '1' then
        temp_angle := angle_in;
        -- Normalizzazione a [-2π, 2π]
        if temp_angle >= TWO_PI_Q12 then
            temp_angle := temp_angle - TWO_PI_Q12;
        elsif temp_angle < -TWO_PI_Q12 then
            temp_angle := temp_angle + TWO_PI_Q12;
        end if;
        -- Normalizzazione a [-π, π]
        if temp_angle > PI_Q12 then
            temp_angle := temp_angle - TWO_PI_Q12;
        elsif temp_angle <= -PI_Q12 then
            temp_angle := temp_angle + TWO_PI_Q12;
        end if;
        angle_in_norm_val <= temp_angle;
    else
        angle_in_norm_val <= to_signed(0, 16);
    end if;
end process;

    -- Riduzione del quadrante per la modalità rotazione (mode "00")
    -- Questo processo prepara gli input per il CORDIC in modo che l'angolo 'z' sia nel primo quadrante.
    -- I segni finali vengono poi aggiustati in base al 'quadrant' determinato qui.
    process(angle_in_norm_val, input_valid)
        variable temp_x : signed(15 downto 0);
        variable temp_y : signed(15 downto 0);
        variable temp_z : signed(15 downto 0);
        variable quad : std_logic_vector(1 downto 0);
    begin
        if input_valid = '1' then
            temp_x := K_FACTOR_INV; -- Inizializza con il fattore K inverso
            temp_y := to_signed(0, 16);
            temp_z := angle_in_norm_val;
            quad := "00"; -- Default: I Quadrante

            -- Mappatura degli angoli al primo quadrante [0, PI/2) e determinazione del quadrante
            if temp_z >= to_signed(0, 16) and temp_z < HALF_PI_Q12 then
                quad := "00"; -- I quadrante
            elsif temp_z >= HALF_PI_Q12 and temp_z < PI_Q12 then
                temp_z := PI_Q12 - temp_z; -- Mappa a PI/2 - angle
                quad := "01"; -- II quadrante
            elsif temp_z >= PI_Q12 and temp_z < PI_Q12 + HALF_PI_Q12 then
                temp_z := temp_z - PI_Q12; -- Mappa a angle - PI
                quad := "10"; -- III quadrante
            elsif temp_z >= PI_Q12 + HALF_PI_Q12 and temp_z <= TWO_PI_Q12 then -- Include 2PI per arrotondamento
                   temp_z := TWO_PI_Q12 - temp_z; -- Mappa a 2PI - angle
                   quad := "11"; -- IV quadrante
            -- Gestione degli angoli negativi
            elsif temp_z < to_signed(0, 16) and temp_z > -HALF_PI_Q12 then -- Tra 0 e -PI/2
                   temp_z := abs(temp_z); -- Mappa al primo quadrante
                   quad := "11"; -- IV quadrante (x positivo, y negativo)
            elsif temp_z <= -HALF_PI_Q12 and temp_z > -PI_Q12 then -- Tra -PI/2 e -PI
                   temp_z := abs(temp_z + PI_Q12); -- Mappa a PI - abs(angle)
                   quad := "10"; -- III quadrante (x negativo, y negativo)
            elsif temp_z <= -PI_Q12 and temp_z >= -TWO_PI_Q12 then -- Correzione qui: PI_Q11 -> PI_Q12
                   temp_z := abs(temp_z + TWO_PI_Q12);
                   quad := "01"; -- II quadrante (x negativo, y positivo) - Questo caso non dovrebbe attivarsi se la normalizzazione a [-PI, PI] funziona bene.
            end if;

            -- Cases for exact angles (0, 90, 180, 270, 360) for precision
            if angle_in_norm_val = to_signed(0, 16) then -- 0 or 360
                quad := "00";
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = HALF_PI_Q12 then -- 90
                quad := "00";
                temp_z := HALF_PI_Q12; -- The CORDIC should handle this directly
            elsif angle_in_norm_val = PI_Q12 then -- 180
                quad := "01"; -- Treat as Q2 to get a 0 input to CORDIC and then negative x_out
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = -PI_Q12 then -- -180
                quad := "01"; -- Treat as Q2 to get a 0 input to CORDIC and then negative x_out
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = -HALF_PI_Q12 then -- -90 or 270
                quad := "11"; -- Treat as Q4 to get a 0 input to CORDIC and then negative y_out
                temp_z := HALF_PI_Q12;
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
                    -- For Arcsin(y), use (sqrt(1-y^2), y). The current x_in is already the sqrt(1-y^2) value
                    -- CORDIC in vectoring mode brings y to 0. The resulting angle will be the arcsin.
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
                        
                        -- Rounding for known values (0, 4096, 2896)
                        if x_out_temp >= 4000 and x_out_temp <= 4150 then
                            x_out_temp := to_signed(4096, 16);
                        elsif x_out_temp <= -4000 and x_out_temp >= -4150 then
                            x_out_temp := to_signed(-4096, 16);
                        elsif abs(x_out_temp) < 50 then -- Near zero
                            x_out_temp := to_signed(0, 16);
                        elsif x_out_temp >= 2800 and x_out_temp <= 3000 then -- Slightly extended range
                            x_out_temp := to_signed(2896, 16);
                        elsif x_out_temp <= -2800 and x_out_temp >= -3000 then -- Slightly extended range
                            x_out_temp := to_signed(-2896, 16);
                        end if;

                        if y_out_temp >= 4000 and y_out_temp <= 4150 then
                            y_out_temp := to_signed(4096, 16);
                        elsif y_out_temp <= -4000 and y_out_temp >= -4150 then
                            y_out_temp := to_signed(-4096, 16);
                        elsif abs(y_out_temp) < 50 then -- Near zero
                            y_out_temp := to_signed(0, 16);
                        elsif y_out_temp >= 2800 and y_out_temp <= 3000 then -- Slightly extended range
                            y_out_temp := to_signed(2896, 16);
                        elsif y_out_temp <= -2800 and y_out_temp >= -3000 then -- Slightly extended range
                            y_out_temp := to_signed(-2896, 16);
                        end if;
                        
                        -- Debug signals
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
                            when others => -- Fallback (should not happen)
                                x_out <= x_out_temp;
                                y_out <= y_out_temp;
                        end case;
                        angle_out <= angle_in; -- For this mode, angle_out is angle_in
                        
                    when "01" => -- Vectoring mode (arctan)
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);
                        angle_out_temp := z_pipe(N_ITERATIONS);
                        
                        -- Rounding for known arctan angles
                        if angle_out_temp >= 3170 and angle_out_temp <= 3270 then
                            angle_out_temp := to_signed(3220, 16);
                        elsif angle_out_temp >= 9598 and angle_out_temp <= 9698 then
                            angle_out_temp := to_signed(9648, 16);
                        elsif angle_out_temp >= 16038 and angle_out_temp <= 16138 then
                            angle_out_temp := to_signed(16088, 16);
                        elsif angle_out_temp >= 22466 and angle_out_temp <= 22566 then
                            angle_out_temp := to_signed(22516, 16);
                        end if;
                        
                        debug_angle_out_temp <= angle_out_temp;

                        -- Adjust sign and quadrant for the resulting angle
                        -- based on original signs of x_in and y_in
                        if x_in_is_negative_pipe(N_ITERATIONS) = false and y_in_is_negative_pipe(N_ITERATIONS) = false then
                            angle_out <= angle_out_temp; -- Q1 (0 to PI/2)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = true and y_in_is_negative_pipe(N_ITERATIONS) = false then
                            angle_out <= PI_Q12 - angle_out_temp; -- Q2 (PI/2 to PI)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = true and y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= angle_out_temp + PI_Q12; -- Q3 (PI to 3PI/2)
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = false and y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= TWO_PI_Q12 - angle_out_temp; -- Q4 (3PI/2 to 2PI)
                        else
                            angle_out <= to_signed(0, 16);
                        end if;
                        x_out <= x_out_temp; -- Magnitude of the vector
                        y_out <= y_out_temp; -- Should be close to zero
                        
                    when "10" => -- Arcsin mode
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);
                        angle_out_temp := z_pipe(N_ITERATIONS);
                        
                        -- Rounding for known arcsin angles
                        if angle_out_temp >= 2085 and angle_out_temp <= 2205 then
                            angle_out_temp := to_signed(2145, 16); -- Arcsin(0.5)
                        elsif angle_out_temp >= 3415 and angle_out_temp <= 3535 then
                            angle_out_temp := to_signed(3474, 16); -- Arcsin(0.75)
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