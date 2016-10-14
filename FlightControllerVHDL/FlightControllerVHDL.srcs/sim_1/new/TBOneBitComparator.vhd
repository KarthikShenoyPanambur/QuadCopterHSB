----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 10.10.2016 20:40:16
-- Design Name: 
-- Module Name: TBOneBitComparator - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity TBOneBitComparator is
--  Port ( );
end TBOneBitComparator;

architecture Behavioral of TBOneBitComparator is
component OneBitComparator is
Port ( i0 : in STD_LOGIC;
           i1 : in STD_LOGIC;
           eq : out STD_LOGIC);
end component;
--for all: use entity work.OneBitComparator(Behavioral)
signal a0,a1,b: std_logic;
begin
uut: OneBitComparator 
             port map(i0=>a0,
                      i1=>a1,
                      eq=>b
                      );
 process
 begin
   a0<='1';
   a1<='0';
 wait for 100ns;
   a0<='0';
   a1<='1';
 wait for 100ns;
   a0<='0';
   a1<='0';
 wait for 100ns;
   a0<='1';
   a1<='1';
wait;
end process;
  

end Behavioral;
