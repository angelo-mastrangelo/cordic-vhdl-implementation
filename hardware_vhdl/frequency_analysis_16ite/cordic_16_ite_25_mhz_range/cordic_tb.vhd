library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity cordic_tb is
end cordic_tb;

architecture test of cordic_tb is
    constant N : integer := 16;
    constant STAGES : integer := 16;
    constant CLK_PERIOD : time := 10 ns;

    signal clk : std_logic := '0';
    signal rst : std_logic := '0';
    signal mode : std_logic_vector(1 downto 0) := "00";
    signal x_in : signed(N-1 downto 0) := (others => '0');
    signal y_in : signed(N-1 downto 0) := (others => '0');
    signal angle_in : signed(N-1 downto 0) := (others => '0');
    signal x_out : signed(N-1 downto 0);
    signal y_out : signed(N-1 downto 0);
    signal angle_out : signed(N-1 downto 0);
    signal done : std_logic;

    constant PI : real := 3.141592653589793;
    constant FRACTIONAL_BITS : integer := 12;
    constant VALUE_SCALE : real := 2.0 ** FRACTIONAL_BITS;
    constant ANGLE_SCALE : real := 2.0 ** FRACTIONAL_BITS;

begin
    uut: entity work.cordic(pipelined)
        port map (
            clk => clk,
            rst => rst,
            mode => mode,
            x_in => x_in,
            y_in => y_in,
            angle_in => angle_in,
            x_out => x_out,
            y_out => y_out,
            angle_out => angle_out,
            done => done
        );

    clk_process: process
    begin
        while true loop
            clk <= '0';
            wait for CLK_PERIOD / 2;
            clk <= '1';
            wait for CLK_PERIOD / 2;
        end loop;
    end process;

    stim_proc: process
        procedure apply_test(
            test_name : string;
            test_mode : std_logic_vector(1 downto 0);
            test_x_in : real;
            test_y_in : real;
            test_angle_in : real;
            expected_x : real := 0.0;
            expected_y : real := 0.0;
            expected_angle : real := 0.0
        ) is
            variable x_scaled : signed(N-1 downto 0);
            variable y_scaled : signed(N-1 downto 0);
            variable angle_scaled : signed(N-1 downto 0);
            variable expected_x_scaled : signed(N-1 downto 0);
            variable expected_y_scaled : signed(N-1 downto 0);
            variable expected_angle_scaled : signed(N-1 downto 0);
            variable mode_str : string(1 to 2);
        begin
            x_scaled := to_signed(integer(test_x_in * VALUE_SCALE), N);
            y_scaled := to_signed(integer(test_y_in * VALUE_SCALE), N);
            angle_scaled := to_signed(integer(test_angle_in * ANGLE_SCALE), N);
            expected_x_scaled := to_signed(integer(expected_x * VALUE_SCALE), N);
            expected_y_scaled := to_signed(integer(expected_y * VALUE_SCALE), N);
            expected_angle_scaled := to_signed(integer(expected_angle * ANGLE_SCALE), N);

            -- Converti test_mode in stringa
            mode_str := std_logic'image(test_mode(1))(2 to 2) & std_logic'image(test_mode(0))(2 to 2);

            report "--- INIZIO " & test_name & " ---";
            report "Inputs: mode = " & mode_str & ", x_in = " & integer'image(to_integer(x_scaled)) &
                   ", y_in = " & integer'image(to_integer(y_scaled)) & ", angle_in = " & integer'image(to_integer(angle_scaled));

            mode <= test_mode;
            x_in <= x_scaled;
            y_in <= y_scaled;
            angle_in <= angle_scaled;

            wait for CLK_PERIOD * 2;

            rst <= '1';
            wait for CLK_PERIOD;
            rst <= '0';
            wait for CLK_PERIOD * (STAGES + 2);

            wait until done = '1' for CLK_PERIOD * (STAGES + 10);
            wait for CLK_PERIOD;

            if done = '1' then
                if test_mode = "00" then
                    report "Output: x_out = " & integer'image(to_integer(x_out)) & ", y_out = " & integer'image(to_integer(y_out));
                    if abs(to_integer(x_out) - to_integer(expected_x_scaled)) <= 4 and abs(to_integer(y_out) - to_integer(expected_y_scaled)) <= 4 then
                        report test_name & " OK: x = " & integer'image(to_integer(x_out)) & ", y = " & integer'image(to_integer(y_out));
                    else
                        report test_name & " FALLITO: x_out = " & integer'image(to_integer(x_out)) & ", Atteso x = " & integer'image(to_integer(expected_x_scaled)) &
                               ", y_out = " & integer'image(to_integer(y_out)) & ", Atteso y = " & integer'image(to_integer(expected_y_scaled)) severity warning;
                    end if;
                else
                    report "Output: angle_out = " & integer'image(to_integer(angle_out));
                    if abs(to_integer(angle_out) - to_integer(expected_angle_scaled)) <= 4 then
                        report test_name & " OK: Valore = " & integer'image(to_integer(angle_out));
                    else
                        report test_name & " FALLITO: Valore = " & integer'image(to_integer(angle_out)) &
                               ", Atteso = " & integer'image(to_integer(expected_angle_scaled)) severity warning;
                    end if;
                end if;
            else
                report test_name & " FALLITO: Nessun output valido (done non ricevuto)" severity warning;
            end if;

            wait for CLK_PERIOD * 2;
        end procedure;

        procedure apply_arcsin_test(
            test_name : string;
            y_val : real;
            expected_angle : real
        ) is
            variable y_scaled : signed(N-1 downto 0);
            variable x_scaled : signed(N-1 downto 0);
            variable y_sq : real;
            variable x_val : real;
            variable expected_angle_scaled : signed(N-1 downto 0);
        begin
            y_scaled := to_signed(integer(y_val * VALUE_SCALE), N);
            y_sq := y_val * y_val;
            x_val := sqrt(1.0 - y_sq);
            x_scaled := to_signed(integer(x_val * VALUE_SCALE), N);
            expected_angle_scaled := to_signed(integer(expected_angle * ANGLE_SCALE), N);

            report "--- INIZIO " & test_name & " ---";
            report "Inputs: mode = 10, x_in = " & integer'image(to_integer(x_scaled)) &
                   ", y_in = " & integer'image(to_integer(y_scaled)) & ", angle_in = 0";

            mode <= "10";
            x_in <= x_scaled;
            y_in <= y_scaled;
            angle_in <= (others => '0');

            wait for CLK_PERIOD * 2;

            rst <= '1';
            wait for CLK_PERIOD;
            rst <= '0';
            wait for CLK_PERIOD * (STAGES + 2);

            wait until done = '1' for CLK_PERIOD * (STAGES + 10);
            wait for CLK_PERIOD;

            if done = '1' then
                report "Output: angle_out = " & integer'image(to_integer(angle_out));
                if abs(to_integer(angle_out) - to_integer(expected_angle_scaled)) <= 4 then
                    report test_name & " OK: Valore = " & integer'image(to_integer(angle_out));
                else
                    report test_name & " FALLITO: Valore = " & integer'image(to_integer(angle_out)) &
                           ", Atteso = " & integer'image(to_integer(expected_angle_scaled)) severity warning;
                end if;
            else
                report test_name & " FALLITO: Nessun output valido (done non ricevuto)" severity warning;
            end if;

            wait for CLK_PERIOD * 2;
        end procedure;

    begin
        rst <= '1';
        mode <= "00";
        x_in <= (others => '0');
        y_in <= (others => '0');
        angle_in <= (others => '0');
        wait for 3 * CLK_PERIOD;
        rst <= '0';
        wait for CLK_PERIOD;

        apply_test("SENO 0°", "00", 1.0, 0.0, 0.0, 1.0, 0.0);
        apply_test("SENO 45°", "00", 1.0, 0.0, PI/4.0, 0.707106781, 0.707106781);
        apply_test("SENO 90°", "00", 1.0, 0.0, PI/2.0, 0.0, 1.0);
        apply_test("SENO 135°", "00", 1.0, 0.0, 3.0*PI/4.0, -0.707106781, 0.707106781);
        apply_test("SENO 180°", "00", 1.0, 0.0, PI, -1.0, 0.0);
        apply_test("SENO 225°", "00", 1.0, 0.0, 5.0*PI/4.0, -0.707106781, -0.707106781);
        apply_test("SENO 270°", "00", 1.0, 0.0, 3.0*PI/2.0, 0.0, -1.0);
        apply_test("SENO 315°", "00", 1.0, 0.0, 7.0*PI/4.0, 0.707106781, -0.707106781);
        apply_test("SENO 360°", "00", 1.0, 0.0, 2.0*PI, 1.0, 0.0);

        apply_test("ARCTAN 45° (1,1)", "01", 1.0, 1.0, 0.0, 0.0, 0.0, PI/4.0);
        apply_test("ARCTAN 135° (-1,1)", "01", -1.0, 1.0, 0.0, 0.0, 0.0, 3.0*PI/4.0);
        apply_test("ARCTAN 225° (-1,-1)", "01", -1.0, -1.0, 0.0, 0.0, 0.0, 5.0*PI/4.0);
        apply_test("ARCTAN 315° (1,-1)", "01", 1.0, -1.0, 0.0, 0.0, 0.0, 7.0*PI/4.0);
        apply_test("ARCTAN (0.5, 0.5)", "01", 0.5, 0.5, 0.0, 0.0, 0.0, PI/4.0);
        apply_test("ARCTAN (-0.25, 0.25)", "01", -0.25, 0.25, 0.0, 0.0, 0.0, 3.0*PI/4.0);

        apply_arcsin_test("ARCSIN(0.5)", 0.5, PI/6.0);
        apply_arcsin_test("ARCSIN(-0.5)", -0.5, -PI/6.0);
        apply_arcsin_test("ARCSIN(-0.75)", -0.75, -0.848062079);

        report "TEST COMPLETATI";
        wait;
    end process;

end test;