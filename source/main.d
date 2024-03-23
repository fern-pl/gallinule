module main;

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
        /* vaddpd(xmm0, xmm1, xmmwordPtr(r11)); */
        /* jmp("a");
        jmp("a");
        loopne("a"); */
        /* lock(add(rax, rbx));
        idsse3();
        aeskeygenassist(xmm2, xmm1, 1); */
        vaeskeygenassist(xmm1, xmm2, 1);
        vaeskeygenassist(xmm0, xmm2, 1);
        mov(rdx, rcx);
        /* vaddpd(xmm4, xmm0, xmm1);
        vaddps(xmm2, xmm1, xmm4);
        addpd(xmm0, xmm2); */
        /* add(rax, qwordPtr(r11));
        add(rax, r11);
        add(qwordPtr(r11), rax);
        add(r11, rax);
        mov(r11, 3); */
    }
    writeln(block.finalize.toHexString);
}