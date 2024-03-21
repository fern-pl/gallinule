module gallinule.main;

import std.stdio;
import gallinule.x86;
import tern.digest : toHexString;
import tern.traits;
import std.traits;

void main()
{
    Block!true block;
    with (block)
    {
        /* vaddpd(xmm0, xmm1, xmm2); */
        vaddpd(xmm0, xmm1, xmmwordPtr(r11));
        /* add(rax, qwordPtr(r11));
        add(rax, r11);
        add(qwordPtr(r11), rax);
        add(r11, rax);
        mov(r11, 3); */
    }
    writeln(block.finalize.toHexString);
}