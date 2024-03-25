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
        andn(rax, rbx, qwordPtr(rbx));
        vaddps(xmm7, xmm1, xmm8);
    }
    writeln(block.finalize.toHexString);
}