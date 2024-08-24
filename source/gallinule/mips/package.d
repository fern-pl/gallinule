// https://s3-eu-west-1.amazonaws.com/downloads-mips/documents/MD00086-2B-MIPS32BIS-AFP-6.06.pdf
module gallinule.mips;

package:
enum S = 0b00000;
enum D = 0b00001;
enum Q = 0b00011;
enum W = 0b00100;
enum L = 0b00101;
enum PS = 0b01000;

public:
enum rzero = 0;
enum rat = 1;
enum rv0 = 2;
enum rv1 = 3;
enum ra0 = 4;
enum ra1 = 5;
enum ra2 = 6;
enum ra3 = 7;
enum rt0 = 8;
enum rt1 = 9;
enum rt2 = 10;
enum rt3 = 11;
enum rt4 = 12;
enum rt5 = 13;
enum rt6 = 14;
enum rt7 = 15;
enum rs0 = 16;
enum rs1 = 17;
enum rs2 = 18;
enum rs3 = 19;
enum rs4 = 20;
enum rs5 = 21;
enum rs6 = 22;
enum rs7 = 23;
enum rt8 = 24;
enum rt9 = 25;
enum rk0 = 26;
enum rk1 = 27;
enum rgp = 28;
enum rsp = 29;
enum rfp = 30;
enum rra = 31;