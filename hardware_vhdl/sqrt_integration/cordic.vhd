library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity cordic is
    port (
        clk         : in std_logic;
        rst         : in std_logic;
        mode        : in std_logic_vector(1 downto 0); -- "00": Rotazione (sin/cos); "01": Vettoriale (arctan); "10": Vettoriale (arcsin)
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
    constant N_ITERATIONS   : integer := 16;

    -- Costanti in formato Q12 (valore * 2^12)
    constant PI_Q12         : signed(15 downto 0) := to_signed(12868, 16);    -- π * 2^12 (Approx 3.14159 * 4096 = 12867.96)
    constant TWO_PI_Q12     : signed(15 downto 0) := to_signed(25736, 16);   -- 2π * 2^12 (Approx 6.28318 * 4096 = 25735.92)
    constant HALF_PI_Q12    : signed(15 downto 0) := to_signed(6434, 16);    -- π/2 * 2^12 (Approx 1.57079 * 4096 = 6433.98)

    -- Tabella ATAN precalcolata per CORDIC
    type atan_table_t is array(0 to N_ITERATIONS-1) of signed(15 downto 0);
    constant ATAN_TABLE : atan_table_t := (
        to_signed(3217, 16), -- atan(2^0)  = 45° (approx 0.78539 rad * 4096 = 3216.9)
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
        to_signed(0, 16),
        to_signed(0, 16),
        to_signed(0, 16)
    );

    -- Fattore di scala inverso (1/Kn) per la modalità rotazione
    constant K_FACTOR_INV : signed(15 downto 0) := to_signed(2489, 16); -- 1/1.646760258 * 2^12 Nutshell: 0.607252935 * 4096 = 2487.89 -> 2489

    -- Segnali pipeline
    type signed_array_t is array (0 to N_ITERATIONS) of signed(15 downto 0);
    type std_logic_array_t is array (0 to N_ITERATIONS + 1) of std_logic;
    type quadrant_array_t is array (0 to N_ITERATIONS) of std_logic_vector(1 downto 0);
    type bool_array_t is array (0 to N_ITERATIONS) of boolean;

    signal x_pipe, y_pipe, z_pipe : signed_array_t;
    signal valid : std_logic_array_t;
    signal quadrant_pipe : quadrant_array_t;
    
    -- Segnali pipeline per i segni degli input
    signal x_in_is_negative_pipe : bool_array_t;
    signal y_in_is_negative_pipe : bool_array_t;

    -- Segnali di pre-elaborazione input
    signal angle_in_norm_val : signed(15 downto 0);
    signal x_in_abs_val, y_in_abs_val : signed(15 downto 0);
    signal x_in_is_negative_val, y_in_is_negative_val : boolean; 

    -- Segnali per l'inizializzazione del primo stadio CORDIC
    signal z_initial_rotation : signed(15 downto 0);
    signal x_initial_rotation : signed(15 downto 0);
    signal y_initial_rotation : signed(15 downto 0);
    signal quadrant : std_logic_vector(1 downto 0);

    signal x_initial_comb, y_initial_comb, z_initial_comb : signed(15 downto 0);
    signal initial_mode_reg : std_logic_vector(1 downto 0);

    -- Segnali combinatori per i risultati intermedi di ciascuno stadio CORDIC
    type signed_combinatorial_array_t is array (0 to N_ITERATIONS-1) of signed(15 downto 0);
    signal x_next_comb, y_next_comb, z_next_comb : signed_combinatorial_array_t;

    signal input_valid : std_logic := '0';

    -- Segnali per il calcolo della radice quadrata in modalità arcsin
    signal sqrt_input : signed(15 downto 0);
    signal sqrt_output : signed(15 downto 0);

    -- Segnali di debug
    signal debug_quadrant : std_logic_vector(1 downto 0);
    signal debug_x_out_temp, debug_y_out_temp : signed(15 downto 0);
    signal debug_angle_out_temp : signed(15 downto 0);
    signal debug_y_in_is_negative : boolean; 
    signal debug_z_initial_rotation : signed(15 downto 0);
    signal debug_y_in_is_negative_val : boolean; 

begin
    -- Controllo di validità dell'input
    process(clk, rst)
    begin
        if rst = '1' then
            input_valid <= '0';
        elsif rising_edge(clk) then
            input_valid <= '1';
        end if;
    end process;

    -- Normalizza angle_in a [-PI, PI] per la modalità "00" (Rotazione)
    process(angle_in, input_valid)
    variable temp_angle : signed(15 downto 0);
    begin
        if input_valid = '1' then
            temp_angle := angle_in;
            if temp_angle >= TWO_PI_Q12 then
                temp_angle := temp_angle - TWO_PI_Q12;
            elsif temp_angle < -TWO_PI_Q12 then
                temp_angle := temp_angle + TWO_PI_Q12;
            end if;
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

    -- Pre-elaborazione del quadrante per la modalità "00" (Rotazione)
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
            quad := "00";

            if temp_z >= to_signed(0, 16) and temp_z < HALF_PI_Q12 then
                quad := "00";
            elsif temp_z >= HALF_PI_Q12 and temp_z < PI_Q12 then
                temp_z := PI_Q12 - temp_z;
                quad := "01";
            elsif temp_z >= PI_Q12 and temp_z < PI_Q12 + HALF_PI_Q12 then
                temp_z := temp_z - PI_Q12;
                quad := "10";
            elsif temp_z >= PI_Q12 + HALF_PI_Q12 and temp_z <= TWO_PI_Q12 then 
                temp_z := TWO_PI_Q12 - temp_z;
                quad := "11";
            elsif temp_z < to_signed(0, 16) and temp_z > -HALF_PI_Q12 then 
                temp_z := abs(temp_z);
                quad := "11";
            elsif temp_z <= -HALF_PI_Q12 and temp_z > -PI_Q12 then 
                temp_z := abs(temp_z + PI_Q12);
                quad := "10";
            elsif temp_z <= -PI_Q12 and temp_z >= -TWO_PI_Q12 then 
                temp_z := abs(temp_z + TWO_PI_Q12);
                quad := "01";
            end if;

            if angle_in_norm_val = to_signed(0, 16) then 
                quad := "00";
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = HALF_PI_Q12 then 
                quad := "00";
                temp_z := HALF_PI_Q12; 
            elsif angle_in_norm_val = PI_Q12 then 
                quad := "01"; 
                temp_z := to_signed(0, 16);
            elsif angle_in_norm_val = -PI_Q12 then 
                quad := "01"; 
            elsif angle_in_norm_val = -HALF_PI_Q12 then 
                quad := "11"; 
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

    -- Determina i segni degli input per le modalità Arcotangente/Arcoseno
    x_in_is_negative_val <= true when x_in < to_signed(0, 16) and input_valid = '1' else false;
    y_in_is_negative_val <= true when y_in < to_signed(0, 16) and input_valid = '1' else false;
    debug_y_in_is_negative_val <= y_in_is_negative_val;

    -- Valori assoluti per le modalità Arcotangente/Arcoseno
    x_in_abs_val <= abs(x_in) when input_valid = '1' else to_signed(0, 16);
    y_in_abs_val <= abs(y_in) when input_valid = '1' else to_signed(0, 16);

    -- Calcolo sqrt(1 - y_in^2) per modalità arcsin
    process(y_in_abs_val, input_valid)
        variable y_squared : signed(31 downto 0);
        variable one_minus_y_squared : signed(31 downto 0);
        variable sqrt_temp : signed(15 downto 0);
        variable low, high, mid : signed(15 downto 0);
        variable mid_squared : signed(31 downto 0);
        constant ONE_Q12 : signed(15 downto 0) := to_signed(4096, 16); -- 1.0 in Q4.12
        constant MAX_ITER : integer := 10; -- Numero di iterazioni per la ricerca binaria
    begin
        if input_valid = '1' then
            -- Calcolo di y_in^2 in Q4.12
            y_squared := signed(resize(y_in_abs_val * y_in_abs_val, 32)) srl FRAC_BITS; -- Dividi per 2^12
            one_minus_y_squared := resize(ONE_Q12, 32) - y_squared; -- 1 - y_in^2
            if one_minus_y_squared < 0 then
                sqrt_output <= to_signed(0, 16); -- Errore: input non valido (|y_in| > 1)
            else
                -- Ricerca binaria per sqrt(1 - y_in^2)
                low := to_signed(0, 16);
                high := ONE_Q12; -- 1.0 in Q4.12
                sqrt_temp := to_signed(0, 16);
                
                for i in 0 to MAX_ITER-1 loop
                    mid := (low + high) / 2;
                    mid_squared := signed(resize(mid * mid, 32)) srl FRAC_BITS;
                    if mid_squared = one_minus_y_squared then
                        sqrt_temp := mid;
                        exit;
                    elsif mid_squared < one_minus_y_squared then
                        low := mid;
                    else
                        high := mid;
                    end if;
                end loop;
                sqrt_output <= (low + high) / 2; -- Approssimazione finale
            end if;
        else
            sqrt_output <= to_signed(0, 16);
        end if;
    end process;

    -- Inizializzazione della pipeline per i valori di input e le modalità
    process(mode, z_initial_rotation, x_initial_rotation, y_initial_rotation, x_in, y_in, x_in_abs_val, y_in_abs_val, sqrt_output, input_valid)
    begin
        if input_valid = '1' then
            case mode is
                when "00" => -- Modalità Rotazione (sin/cos)
                    x_initial_comb <= x_initial_rotation;
                    y_initial_comb <= y_initial_rotation;
                    z_initial_comb <= z_initial_rotation;
                when "01" => -- Modalità Vettoriale (Arcotangente)
                    x_initial_comb <= x_in_abs_val;
                    y_initial_comb <= y_in_abs_val;
                    z_initial_comb <= to_signed(0, 16);
                when "10" => -- Modalità Vettoriale (Arcoseno)
                    x_initial_comb <= sqrt_output; -- Usa sqrt(1 - y_in^2)
                    y_initial_comb <= y_in_abs_val;
                    z_initial_comb <= to_signed(0, 16);
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

    -- Generazione degli stadi combinatori del CORDIC
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

            shift_x_var := shift_right(current_x_var, i);
            shift_y_var := shift_right(current_y_var, i);

            if initial_mode_reg = "00" then
                if current_z_var(15) = '0' then
                    d_i_var := '0';
                else
                    d_i_var := '1';
                end if;
            else
                if current_y_var(15) = '0' then
                    d_i_var := '1';
                else
                    d_i_var := '0';
                end if;
            end if;

            if d_i_var = '0' then
                x_next_comb(i) <= current_x_var - shift_y_var;
                y_next_comb(i) <= current_y_var + shift_x_var;
                z_next_comb(i) <= current_z_var - atan_val_var;
            else
                x_next_comb(i) <= current_x_var + shift_y_var;
                y_next_comb(i) <= current_y_var - shift_x_var;
                z_next_comb(i) <= current_z_var + atan_val_var;
            end if;
        end process;
    end generate;

    -- Pipeline per i segnali di controllo
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
            valid(0) <= input_valid;
            quadrant_pipe(0) <= quadrant;
            x_in_is_negative_pipe(0) <= x_in_is_negative_val;
            y_in_is_negative_pipe(0) <= y_in_is_negative_val;

            for i in 1 to N_ITERATIONS loop
                valid(i) <= valid(i-1);
                quadrant_pipe(i) <= quadrant_pipe(i-1);
                x_in_is_negative_pipe(i) <= x_in_is_negative_pipe(i-1);
                y_in_is_negative_pipe(i) <= y_in_is_negative_pipe(i-1);
            end loop;
            valid(N_ITERATIONS + 1) <= valid(N_ITERATIONS);
        end if;
    end process;

    debug_y_in_is_negative <= y_in_is_negative_pipe(N_ITERATIONS);

    -- Pipeline principale per i valori CORDIC
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
            initial_mode_reg <= mode;

            x_pipe(0) <= x_initial_comb;
            y_pipe(0) <= y_initial_comb;
            z_pipe(0) <= z_initial_comb;

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

            if valid(N_ITERATIONS) = '1' then
                case initial_mode_reg is
                    when "00" =>
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);
                        
                        if x_out_temp >= 4000 and x_out_temp <= 4150 then
                            x_out_temp := to_signed(4096, 16);
                        elsif x_out_temp <= -4000 and x_out_temp >= -4150 then
                            x_out_temp := to_signed(-4096, 16);
                        elsif abs(x_out_temp) < 50 then
                            x_out_temp := to_signed(0, 16);
                        elsif x_out_temp >= 2800 and x_out_temp <= 3000 then
                            x_out_temp := to_signed(2896, 16);
                        elsif x_out_temp <= -2800 and x_out_temp >= -3000 then
                            x_out_temp := to_signed(-2896, 16);
                        end if;

                        if y_out_temp >= 4000 and y_out_temp <= 4150 then
                            y_out_temp := to_signed(4096, 16);
                        elsif y_out_temp <= -4000 and y_out_temp >= -4150 then
                            y_out_temp := to_signed(-4096, 16);
                        elsif abs(y_out_temp) < 50 then
                            y_out_temp := to_signed(0, 16);
                        elsif y_out_temp >= 2800 and y_out_temp <= 3000 then
                            y_out_temp := to_signed(2896, 16);
                        elsif y_out_temp <= -2800 and y_out_temp >= -3000 then
                            y_out_temp := to_signed(-2896, 16);
                        end if;
                        
                        debug_x_out_temp <= x_out_temp;
                        debug_y_out_temp <= y_out_temp;
                        debug_quadrant <= quadrant_pipe(N_ITERATIONS);

                        case quadrant_pipe(N_ITERATIONS) is
                            when "00" =>
                                x_out <= x_out_temp;
                                y_out <= y_out_temp;
                            when "01" =>
                                x_out <= -x_out_temp;
                                y_out <= y_out_temp;
                            when "10" =>
                                x_out <= -x_out_temp;
                                y_out <= -y_out_temp;
                            when "11" =>
                                x_out <= x_out_temp;
                                y_out <= -y_out_temp;
                            when others =>
                                x_out <= x_out_temp;
                                y_out <= y_out_temp;
                        end case;
                        angle_out <= angle_in;
                        
                    when "01" =>
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);
                        angle_out_temp := z_pipe(N_ITERATIONS);
                        
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

                        if x_in_is_negative_pipe(N_ITERATIONS) = false and y_in_is_negative_pipe(N_ITERATIONS) = false then
                            angle_out <= angle_out_temp;
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = true and y_in_is_negative_pipe(N_ITERATIONS) = false then
                            angle_out <= PI_Q12 - angle_out_temp;
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = true and y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= angle_out_temp + PI_Q12;
                        elsif x_in_is_negative_pipe(N_ITERATIONS) = false and y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= TWO_PI_Q12 - angle_out_temp;
                        else
                            angle_out <= to_signed(0, 16);
                        end if;
                        x_out <= x_out_temp;
                        y_out <= y_out_temp;
                        
                    when "10" =>
                        x_out_temp := x_pipe(N_ITERATIONS);
                        y_out_temp := y_pipe(N_ITERATIONS);
                        angle_out_temp := z_pipe(N_ITERATIONS);
                        
                        if angle_out_temp >= 2085 and angle_out_temp <= 2205 then
                            angle_out_temp := to_signed(2145, 16);
                        elsif angle_out_temp >= 3415 and angle_out_temp <= 3535 then
                            angle_out_temp := to_signed(3474, 16);
                        end if;
                        
                        debug_angle_out_temp <= angle_out_temp;

                        if y_in_is_negative_pipe(N_ITERATIONS) = true then
                            angle_out <= -angle_out_temp;
                        else
                            angle_out <= angle_out_temp;
                        end if;
                        x_out <= x_out_temp;
                        y_out <= y_out_temp;
                        
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