module gallinule.x86;

import std.bitmanip;
import std.traits;
import std.typecons;
import tern.algorithm;

public struct Reg(int SIZE)
{
public:
final:
    ubyte index;
    bool extended;
}

public struct Encodable(T)
{
    T value;
    alias value this;
}

Encodable!T encodable(T)(T val)
{
    return Encodable!T(val);
}

public enum cr0 = Reg!(-1)(0);
public enum cr2 = Reg!(-1)(2);
public enum cr3 = Reg!(-1)(3);
public enum cr4 = Reg!(-1)(4);

public enum dr0 = Reg!(-2)(0);
public enum dr1 = Reg!(-2)(1);
public enum dr2 = Reg!(-2)(2);
public enum dr3 = Reg!(-2)(3);
public enum dr6 = Reg!(-2)(6);
public enum dr7 = Reg!(-2)(7);

public enum al = Reg!8(0);
public enum cl = Reg!8(1);
public enum dl = Reg!8(2);
public enum bl = Reg!8(3);
public enum ah = Reg!8(4);
public enum ch = Reg!8(5);
public enum dh = Reg!8(6);
public enum bh = Reg!8(7);
public enum spl = Reg!8(4, true);
public enum bpl = Reg!8(5, true);
public enum sil = Reg!8(6, true);
public enum dil = Reg!8(7, true);
public enum r8b = Reg!8(8);
public enum r9b = Reg!8(9);
public enum r10b = Reg!8(10);
public enum r11b = Reg!8(11);
public enum r12b = Reg!8(12);
public enum r13b = Reg!8(13);
public enum r14b = Reg!8(14);
public enum r15b = Reg!8(15);

public enum ax = Reg!16(0);
public enum cx = Reg!16(1);
public enum dx = Reg!16(2);
public enum bx = Reg!16(3);
public enum sp = Reg!16(4);
public enum bp = Reg!16(5);
public enum si = Reg!16(6);
public enum di = Reg!16(7);
public enum r8w = Reg!16(8);
public enum r9w = Reg!16(9);
public enum r10w = Reg!16(10);
public enum r11w = Reg!16(11);
public enum r12w = Reg!16(12);
public enum r13w = Reg!16(13);
public enum r14w = Reg!16(14);
public enum r15w = Reg!16(15);

public enum eax = Reg!32(0);
public enum ecx = Reg!32(1);
public enum edx = Reg!32(2);
public enum ebx = Reg!32(3);
public enum esp = Reg!32(4);
public enum ebp = Reg!32(5);
public enum esi = Reg!32(6);
public enum edi = Reg!32(7);
public enum r8d = Reg!32(8);
public enum r9d = Reg!32(9);
public enum r10d = Reg!32(10);
public enum r11d = Reg!32(11);
public enum r12d = Reg!32(12);
public enum r13d = Reg!32(13);
public enum r14d = Reg!32(14);
public enum r15d = Reg!32(15);

public enum rax = Reg!64(0);
public enum rcx = Reg!64(1);
public enum rdx = Reg!64(2);
public enum Rbx = Reg!64(3);
public enum rsp = Reg!64(4);
public enum rbp = Reg!64(5);
public enum rsi = Reg!64(6);
public enum rdi = Reg!64(7);
public enum r8 = Reg!64(8);
public enum r9 = Reg!64(9);
public enum r10 = Reg!64(10);
public enum r11 = Reg!64(11);
public enum r12 = Reg!64(12);
public enum r13 = Reg!64(13);
public enum r14 = Reg!64(14);
public enum r15 = Reg!64(15);

public enum mmx0 = Reg!64(0);
public enum mmx1 = Reg!64(1);
public enum mmx2 = Reg!64(2);
public enum mmx3 = Reg!64(3);
public enum mmx4 = Reg!64(4);
public enum mmx5 = Reg!64(5);
public enum mmx6 = Reg!64(6);
public enum mmx7 = Reg!64(7);
public enum mmx8 = Reg!64(8);
public enum mmx9 = Reg!64(9);
public enum mmx10 = Reg!64(10);
public enum mmx11 = Reg!64(11);
public enum mmx12 = Reg!64(12);
public enum mmx13 = Reg!64(13);
public enum mmx14 = Reg!64(14);
public enum mmx15 = Reg!64(15);

public enum xmm0 = Reg!128(0);
public enum xmm1 = Reg!128(1);
public enum xmm2 = Reg!128(2);
public enum xmm3 = Reg!128(3);
public enum xmm4 = Reg!128(4);
public enum xmm5 = Reg!128(5);
public enum xmm6 = Reg!128(6);
public enum xmm7 = Reg!128(7);
public enum xmm8 = Reg!128(8);
public enum xmm9 = Reg!128(9);
public enum xmm10 = Reg!128(10);
public enum xmm11 = Reg!128(11);
public enum xmm12 = Reg!128(12);
public enum xmm13 = Reg!128(13);
public enum xmm14 = Reg!128(14);
public enum xmm15 = Reg!128(15);

public enum ymm0 = Reg!256(0);
public enum ymm1 = Reg!256(1);
public enum ymm2 = Reg!256(2);
public enum ymm3 = Reg!256(3);
public enum ymm4 = Reg!256(4);
public enum ymm5 = Reg!256(5);
public enum ymm6 = Reg!256(6);
public enum ymm7 = Reg!256(7);
public enum ymm8 = Reg!256(8);
public enum ymm9 = Reg!256(9);
public enum ymm10 = Reg!256(10);
public enum ymm11 = Reg!256(11);
public enum ymm12 = Reg!256(12);
public enum ymm13 = Reg!256(13);
public enum ymm14 = Reg!256(14);
public enum ymm15 = Reg!256(15);

public enum zmm0 = Reg!512(0);
public enum zmm1 = Reg!512(1);
public enum zmm2 = Reg!512(2);
public enum zmm3 = Reg!512(3);
public enum zmm4 = Reg!512(4);
public enum zmm5 = Reg!512(5);
public enum zmm6 = Reg!512(6);
public enum zmm7 = Reg!512(7);
public enum zmm8 = Reg!512(8);
public enum zmm9 = Reg!512(9);
public enum zmm10 = Reg!512(10);
public enum zmm11 = Reg!512(11);
public enum zmm12 = Reg!512(12);
public enum zmm13 = Reg!512(13);
public enum zmm14 = Reg!512(14);
public enum zmm15 = Reg!512(15);

public enum st0 = Reg!(-3)(0);
public enum st1 = Reg!(-3)(1);
public enum st2 = Reg!(-3)(2);
public enum st3 = Reg!(-3)(3);
public enum st4 = Reg!(-3)(4);
public enum st5 = Reg!(-3)(5);
public enum st6 = Reg!(-3)(6);
public enum st7 = Reg!(-3)(7);

public enum ubyte es = 0x26;
public enum ubyte cs = 0x2e;
public enum ubyte ss = 0x36;
public enum ubyte ds = 0x3e;
public enum ubyte fs = 0x64;
public enum ubyte gs = 0x65;

private enum int NA = 0xdebfeeda;
private enum int NP = 0xdeedbeaf;
private enum int VEX = 0xdaedbfee;
private enum int VEXI = 0xdaedbfea;

// map_select
private enum ubyte DEFAULT = 1;
private enum ubyte XOP = 0;
private enum ubyte MSR = 7;

public struct Address(int SIZE)
{
public:
final:
    bool direct;
    bool override32;
    ubyte register;
    uint offset;
    ubyte segment = ds;

    this(T)(T register, ubyte segment, uint offset = 0)
        if (isInstanceOf!(Reg, T))
    {
        if (!is(T == Reg!64))
            override32 = true;

        this.direct = true;
        this.register = register.index;
        this.offset = offset;
        this.segment = segment;
    }

    this(T)(T register, uint offset = 0)
        if (isInstanceOf!(Reg, T))
    {
        if (!is(T == Reg!64))
            override32 = true;

        this.direct = true;
        this.register = register.index;
        this.offset = offset;
    }

    this(uint offset, ubyte segment = ds)
    {
        // TODO: Why? Is this even correct??
        this.register = 4;
        this.offset = offset;
        this.segment = segment;
    }
}

public enum Mode
{
    Memory,
    MemoryOffset8,
    MemoryOffsetExt,
    Register
}

// TODO: Big endian?
public union ModRM
{
public:
final:
    struct
    {
        mixin(bitfields!(
            ubyte, "src", 3,
            ubyte, "dst", 3,
            ubyte, "mod", 2
        ));
    }
    ubyte b;
    alias b this;
}

public static ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst)
    if (isInstanceOf!(Address, SRC) && isInstanceOf!(Reg, DST))
{
    if (!src.direct)
        // TODO: Using 0x25 as the SIB every time is evil!!
        return generateModRM!OP(DST(src.register), dst, Mode.Memory)~0x25~(cast(ubyte*)&src.offset)[0..uint.sizeof];
    else
    {
        if (src.offset == 0)
            return generateModRM!OP(DST(src.register), dst, Mode.Memory);
        else
        {
            if (src.offset >= ubyte.max)
                return generateModRM!OP(DST(src.register), dst, Mode.MemoryOffset8)~cast(ubyte)src.offset;
            else
                return generateModRM!OP(DST(src.register), dst, Mode.MemoryOffsetExt)~(cast(ubyte*)&src.offset)[0..uint.sizeof];
        }
    }
}

public static ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst, Mode mod = Mode.Register)
    if (isInstanceOf!(Reg, SRC) && isInstanceOf!(Reg, DST))
{
    ModRM generateModRM;
    generateModRM.src = (src.index % 8);
    generateModRM.dst = (dst.index % 8) | OP;
    generateModRM.mod = cast(ubyte)mod;
    return [generateModRM];
}

public static ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst)
    if (isInstanceOf!(Address, SRC) && isInstanceOf!(Address, DST))
{
    return generateModRM!OP(Reg!(TemplateArgsOf!(DST))(dst.register), Reg!(TemplateArgsOf!(SRC))(src.register));
}

public static ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst)
    if (isInstanceOf!(Reg, SRC) && isInstanceOf!(Address, DST))
{
    return generateModRM!OP(dst, src);
}

// TODO: SIB

public struct Label
{
public:
final:
    ptrdiff_t location;
    string name;
}

public enum isRM(T, int SIZE) = is(T == Reg!SIZE) || is(T == Address!SIZE);

ubyte[][string] branchMap = [
    "jmp1": [0xeb],
    "jmp2": [0xe9],
    "jmp4": [0xe9],
    "ja1": [0x77],
    "jae1": [0x73],
    "jb1": [0x72],
    "jbe1": [0x76],
    "jc1": [0x72],
    "jecxz1": [0xE3],
    "jecxz1": [0xE3],
    "jrcxz1": [0xE3],
    "je1": [0x74],
    "jg1": [0x7F],
    "jge1": [0x7D],
    "jl1": [0x7C],
    "jle1": [0x7E],
    "jna1": [0x76],
    "jnae1": [0x72],
    "jnb1": [0x73],
    "jnbe1": [0x77],
    "jnc1": [0x73],
    "jne1": [0x75],
    "jng1": [0x7E],
    "jnge1": [0x7C],
    "jnl1": [0x7D],
    "jnle1": [0x7F],
    "jno1": [0x71],
    "jnp1": [0x7B],
    "jns1": [0x79],
    "jnz1": [0x75],
    "jo1": [0x70],
    "jp1": [0x7A],
    "jpe1": [0x7A],
    "jpo1": [0x7B],
    "js1": [0x78],
    "jz1": [0x74],
    "ja2": [0x0F, 0x87],
    "ja4": [0x0F, 0x87],
    "jae2": [0x0F, 0x83],
    "jae4": [0x0F, 0x83],
    "jb2": [0x0F, 0x82],
    "jb4": [0x0F, 0x82],
    "jbe2": [0x0F, 0x86],
    "jbe4": [0x0F, 0x86],
    "jc2": [0x0F, 0x82],
    "jc4": [0x0F, 0x82],
    "je2": [0x0F, 0x84],
    "je4": [0x0F, 0x84],
    "jz2": [0x0F, 0x84],
    "jz4": [0x0F, 0x84],
    "jg2": [0x0F, 0x8F],
    "jg4": [0x0F, 0x8F],
    "jge2": [0x0F, 0x8D],
    "jge4": [0x0F, 0x8D],
    "jl2": [0x0F, 0x8C],
    "jl4": [0x0F, 0x8C],
    "jle2": [0x0F, 0x8E],
    "jle4": [0x0F, 0x8E],
    "jna2": [0x0F, 0x86],
    "jna4": [0x0F, 0x86],
    "jnae2": [0x0F, 0x82],
    "jnae4": [0x0F, 0x82],
    "jnb2": [0x0F, 0x83],
    "jnb4": [0x0F, 0x83],
    "jnbe2": [0x0F, 0x87],
    "jnbe4": [0x0F, 0x87],
    "jnc2": [0x0F, 0x83],
    "jnc4": [0x0F, 0x83],
    "jne2": [0x0F, 0x85],
    "jne4": [0x0F, 0x85],
    "jng2": [0x0F, 0x8E],
    "jng4": [0x0F, 0x8E],
    "jnge2": [0x0F, 0x8C],
    "jnge4": [0x0F, 0x8C],
    "jnl2": [0x0F, 0x8D],
    "jnl4": [0x0F, 0x8D],
    "jnle2": [0x0F, 0x8F],
    "jnle4": [0x0F, 0x8F],
    "jno2": [0x0F, 0x81],
    "jno4": [0x0F, 0x81],
    "jnp2": [0x0F, 0x8B],
    "jnp4": [0x0F, 0x8B],
    "jns2": [0x0F, 0x89],
    "jns4": [0x0F, 0x89],
    "jnz2": [0x0F, 0x85],
    "jnz4": [0x0F, 0x85],
    "jo2": [0x0F, 0x80],
    "jo4": [0x0F, 0x80],
    "jp2": [0x0F, 0x8A],
    "jp4": [0x0F, 0x8A],
    "jpe2": [0x0F, 0x8A],
    "jpe4": [0x0F, 0x8A],
    "jpo2": [0x0F, 0x8B],
    "jpo4": [0x0F, 0x8B],
    "js2": [0x0F, 0x88],
    "js4": [0x0F, 0x88],
    "jz2": [0x0F, 0x84],
    "jz4": [0x0F, 0x84],
    "loop1": [0xe2],
    "loope1": [0xe1],
    "loopne1": [0xe0]
];

// TODO: Attributes
public struct Block(bool X64)
{
package:
final:
    Label[] labels;
    Tuple!(ptrdiff_t, string, string)[] branches;
    ubyte[] buffer;

public:
    ubyte[] finalize()
    {
        ptrdiff_t abs;
        foreach (branch; branches)
        {
            ubyte[] buffer;
            Label label = labels.filter!(x => x.name == branch[1])[0];

            branch[0] += abs;
            auto rel = label.location - branch[0];
            bool isRel8 = rel <= byte.max && rel >= byte.min;
            bool isRel16 = rel <= short.max && rel >= short.min;

            buffer ~= branchMap[branch[2]~(isRel8 ? '1' : isRel16 ? '2' : '4')];

            if (rel < 0 && abs < 2)
                rel -= abs | (buffer.length + (isRel8 ? 1 : isRel16 ? 2 : 4));

            if (isRel8)
                buffer ~= cast(ubyte)rel;
            else if (isRel16)
                buffer ~= (cast(ubyte*)&rel)[0..2];
            else
                buffer ~= (cast(ubyte*)&rel)[0..4];

            this.buffer = this.buffer[0..branch[0]]~buffer~this.buffer[branch[0]..$];
            abs += buffer.length;
        }
        branches = null;
        return this.buffer;
    }

    size_t emit(ubyte OP, int SELECTOR, ARGS...)(ARGS args)
    {
        ubyte[] buffer;
        bool prefixed;
        ptrdiff_t ct = (SELECTOR != NA && SELECTOR != NP) ? 3 : 0;
        
        static assert(SELECTOR == NA || SELECTOR == NP || ARGS.length >= 3, "Vector selected instructions must have at least 3 emitted arguments!");

        bool isRM1(size_t INDEX)()
        {
            static if (INDEX >= ARGS.length)
                return false;
            else static if (INDEX + 1 >= ARGS.length)
                return isInstanceOf!(Reg, ARGS[INDEX]) || isInstanceOf!(Address, ARGS[INDEX]);
            else
            {
                return (isInstanceOf!(Reg, ARGS[INDEX]) || isInstanceOf!(Address, ARGS[INDEX])) &&
                    !isInstanceOf!(Reg, ARGS[INDEX + 1]) && !isInstanceOf!(Address, ARGS[INDEX + 1]);
            }
        }
        
        bool isRM2(size_t INDEX)()
        {
            static if (INDEX + 1 >= ARGS.length)
                return false;
            else
                return (isInstanceOf!(Reg, ARGS[INDEX]) || isInstanceOf!(Address, ARGS[INDEX])) && isRM1!(INDEX + 1);
        }
        
        bool isRM3(size_t INDEX)()
        {
            static if (INDEX + 2 >= ARGS.length)
                return false;
            else
                return (isInstanceOf!(Reg, ARGS[INDEX]) || isInstanceOf!(Address, ARGS[INDEX])) &&
                    (isInstanceOf!(Reg, ARGS[INDEX + 1]) || isInstanceOf!(Address, ARGS[INDEX + 1])) && isRM1!(INDEX + 2);
        }

        static if (SELECTOR == NA)
        void generatePrefix(SRC, DST, STOR = int)(SRC src, DST dst, STOR stor = STOR.init)
        {
            prefixed = true;
            bool hasRex;
            bool w;
            bool r;
            bool x;
            bool b;

            static if (isInstanceOf!(Reg, SRC))
            {
                hasRex |= is(SRC == Reg!64) || (is(SRC == Reg!8) && src.extended) || src.index >= 8;
                w = is(SRC == Reg!64);
                b = src.index >= 8;
            }
            else static if (isInstanceOf!(Address, SRC))
            {
                hasRex |= src.register >= 8;
                w = is(SRC == Address!64);
                b = src.register >= 8;
            }
            
            static if (isInstanceOf!(Reg, DST))
            {
                hasRex |= is(DST == Reg!64) || (is(DST == Reg!8) && dst.extended) || dst.index >= 8;
                w = is(DST == Reg!64);
                b = dst.index >= 8;
            }
            else static if (isInstanceOf!(Address, DST))
            {
                hasRex |= dst.register >= 8;
                w = is(DST == Address!64);
                x = dst.register >= 8;
            }

            static if (isInstanceOf!(Address, SRC))
            {
                if (src.segment != ds)
                    buffer = src.segment~buffer;
            }
            else static if (isInstanceOf!(Address, DST))
            {
                if (dst.segment != ds)
                    buffer = dst.segment~buffer;
            }

            if (hasRex)
            {
                ubyte rex = 0b01000000;
                if (w) rex |= (1 << 3);
                if (r) rex |= (1 << 2);
                if (x) rex |= (1 << 1);
                if (b) rex |= (1 << 0);
                
                // Preserve prefixes
                // TODO: Do elsewhere?
                size_t pos = 0;
                foreach (i; 0..5)
                {
                    if (buffer[pos] == 0xf2)
                        pos++;
                    else if (buffer[pos] == 0xf3)
                        pos++;
                    else if (buffer[pos] == 0xf0)
                        pos++;
                    else if (buffer[pos] == 0x66)
                        pos++;
                    else if (buffer[pos] == 0x67)
                        pos++;
                }
                buffer = buffer[0..pos]~rex~buffer[pos..$];
            }

            static if (is(SRC == Reg!16) || is(DST == Reg!16))
                buffer = 0x66~buffer;
            
            static if (X64 && isInstanceOf!(Address, SRC))
            {
                if (src.override32)
                    buffer = 0x67~buffer;
            }

            static if (X64 && isInstanceOf!(Address, DST))
            {
                if (dst.override32)
                    buffer = 0x67~buffer;
            }
        }

        static if (SELECTOR == NP)
        void generatePrefix(SRC, DST, STOR = int)(SRC src, DST dst, STOR stor = STOR.init) { }

        static if (SELECTOR == VEX || SELECTOR == VEXI)
        void generatePrefix(SRC, DST, STOR = int)(SRC src, DST dst, STOR stor = STOR.init)
        {
            prefixed = true;
            bool r;
            bool x;
            bool b;
            ubyte map_select = cast(ubyte)args[1];
            bool we = SELECTOR == VEX;
            ubyte vvvv = 0b1111;
            bool l = args[0] != 128;
            ubyte pp;

            if (args[2] == 0x66)
                pp = 1;
            else if (args[2] == 0xf3)
                pp = 2;
            else if (args[2] == 0xf2)
                pp = 3;

            // TODO: XOP prefix
            static if (isInstanceOf!(Reg, STOR))
            {
                static if (isInstanceOf!(Reg, DST))
                    vvvv = cast(ubyte)~dst.index;
                else static if (isInstanceOf!(Address, DST))
                    vvvv = cast(ubyte)~dst.register;

                dst = DST(stor.index);
            }
            else static if (isInstanceOf!(Address, STOR))
            {
                static if (isInstanceOf!(Reg, DST))
                    vvvv = cast(ubyte)~dst.index;
                else static if (isInstanceOf!(Address, DST))
                    vvvv = cast(ubyte)~dst.register;
                    
                dst = DST(stor.register);
            }

            static if (isInstanceOf!(Reg, SRC))
            {
                static if (SELECTOR == VEXI)
                    we = is(SRC == REG!64);
                r = src.index >= 8;
            }
            else static if (isInstanceOf!(Address, SRC))
            {
                static if (SELECTOR == VEXI)
                    we = is(SRC == Address!64);
                x = src.register >= 8;
            }
            
            static if (isInstanceOf!(Reg, DST))
            {
                static if (SELECTOR == VEXI)
                    we = is(DST == Reg!64);
                b = dst.index >= 8;
            }
            else static if (isInstanceOf!(Address, DST))
            {
                static if (SELECTOR == VEXI)
                    we = is(DST == Address!64);
                b = dst.register >= 8;
            }

            static if (isInstanceOf!(Address, SRC))
            {
                if (src.segment != ds)
                    buffer = src.segment~buffer;
            }
            else static if (isInstanceOf!(Address, DST))
            {
                if (dst.segment != ds)
                    buffer = dst.segment~buffer;
            }

            ubyte[] vex;
            if (map_select != 1 || r || x || b)
            {
                vex ~= 0xc4;
                vex ~= (cast(ubyte)(((r ? 0 : 1) << 6) | ((x ? 0 : 1) << 7) | ((b ? 0 : 1) << 8))) | (map_select & 0b00011111);
            }
            else
                vex ~= 0xc5;
            vex ~= (we ? 0 : 1) << 7 | (vvvv & 0b00001111) << 3 | (l ? 1 : 0) << 2 | (pp & 0b00000011);
            
            size_t pos = 0;
            foreach (i; 0..5)
            {
                if (buffer[pos] == 0xf2)
                    pos++;
                else if (buffer[pos] == 0xf3)
                    pos++;
                else if (buffer[pos] == 0xf0)
                    pos++;
                else if (buffer[pos] == 0x66)
                    pos++;
                else if (buffer[pos] == 0x67)
                    pos++;
            }
            buffer = buffer[0..pos]~vex~buffer[pos..$];
            
            static if (X64 && isInstanceOf!(Address, SRC))
            {
                if (src.override32)
                    buffer = 0x67~buffer;
            }

            static if (X64 && isInstanceOf!(Address, DST))
            {
                if (dst.override32)
                    buffer = 0x67~buffer;
            }
        }

        foreach (i, arg; args)
        {
            if (ct-- > 0)
                continue;

            static if (is(typeof(arg) == int))
                buffer ~= cast(ubyte)arg;
            else static if (is(typeof(arg) == long))
                buffer ~= (cast(ubyte*)&arg)[0..uint.sizeof];
            else static if (isScalarType!(typeof(arg)))
                buffer ~= (cast(ubyte*)&arg)[0..typeof(arg).sizeof];
            else static if (is(typeof(arg) == ubyte[]))
                buffer ~= arg;
            else static if (isInstanceOf!(Encodable, typeof(arg)))
            {
                buffer[$-1] += arg.index % 8;
                generatePrefix(Reg!(TemplateArgsOf!(typeof(arg.value)))(0), arg.value);
            }
            else static if (isRM1!i)
            {
                static if (isInstanceOf!(Reg, typeof(arg)))
                {
                    auto dst = arg;
                    auto src = Reg!(TemplateArgsOf!(typeof(arg)))(0);
                }
                else
                {
                    auto dst = Reg!(TemplateArgsOf!(typeof(arg)))(0);
                    auto src = arg;

                }
                buffer ~= generateModRM!OP(dst, src);
                generatePrefix(src, dst);
            }
            else static if (isRM2!i)
            {
                static if (isInstanceOf!(Reg, typeof(arg)))
                {
                    auto dst = arg;
                    auto src = args[i + 1];
                }
                else
                {
                    auto dst = args[i + 1];
                    auto src = arg;
                }
                buffer ~= generateModRM!OP(dst, src);
                generatePrefix(src, dst);
                ct = 1;
            }
            else static if (isRM3!i)
            {
                static if (isInstanceOf!(Reg, typeof(arg)))
                {
                    auto dst = args[i + 2];
                    auto src = arg;
                }
                else
                {
                    auto dst = arg;
                    auto src = args[i + 2];
                }
                buffer ~= generateModRM!OP(dst, src);
                generatePrefix(src, args[i + 1], dst);
                ct = 2;
            }
            else
                static assert(0, "May not emit a non-scalar, non-ubyte[] value of type '"~typeof(arg).stringof~"'!");
        }

        if (!prefixed)
        {
            static if (SELECTOR != NA || SELECTOR != NP)
                generatePrefix(Reg!(typeof(args[0]).sizeof * 128)(0), Reg!(typeof(args[0]).sizeof * 128)(0));

            static if (SELECTOR == NA || SELECTOR == NP)
            foreach (i, arg; args)
            {
                static if (!is(typeof(arg) == int))
                {
                    static if (args.length - i - 1 == 0)
                        generatePrefix(Reg!(typeof(arg).sizeof * 8)(0), Reg!(typeof(arg).sizeof * 8)(0));
                    else static if (args.length - i - 1 == 1)
                        generatePrefix(Reg!(typeof(arg).sizeof * 8)(0), Reg!(typeof(args[i + 1]).sizeof * 8)(0));
                    else static if (args.length - i - 1 == 2)
                        generatePrefix(Reg!(typeof(arg).sizeof * 8)(0), Reg!(typeof(args[i + 1]).sizeof * 8)(0), Reg!(typeof(args[i + 2]).sizeof * 8)(0));
                    break;
                }
            }
        }

        this.buffer ~= buffer;
        return buffer.length;
    }
    
    // TODO: Fix parameter names to make them more clear
    auto label(string name) => labels ~= Label(buffer.length, name);
     
    auto vaddpd(T)(Reg!128 dst, Reg!128 src, T stor) if (isRM!(T, 128)) => emit!(0, VEX)(128, DEFAULT, 0x66, 0x58, dst, src, stor);

    auto rdtsc() => emit!(0, NA)(0x0f, 0x31);
    auto rdtscp() => emit!(0, NA)(0x0f, 0x01, 0xf9);

    auto cmpxchg(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0xb0, dst, src);
    auto cmpxchg(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xb1, dst, src);
    auto cmpxchg(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xb1, dst, src);
    auto cmpxchg(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xb1, dst, src);

    auto cmpxchg8b(Address!64 dst) => emit!(1, NP)(0x0f, 0xc7, dst);
    auto cmpxchg16b(Address!128 dst) => emit!(1, NA)(0x48, 0x0f, 0xc7, dst);

    auto aaa() => emit!(0, NA)(0x37);
    auto aad() => emit!(0, NA)(0xd5, 0x0a);
    auto aad(ubyte imm8) => emit!(0, NA)(0xd5, imm8);
    auto aam() => emit!(0, NA)(0xd4, 0x0a);
    auto aam(ubyte imm8) => emit!(0, NA)(0xd4, imm8);
    auto aas() => emit!(0, NA)(0x3f);

    auto add(ubyte imm8) => emit!(0, NA)(0x04, imm8);
    auto add(ushort imm16) => emit!(0, NA)(0x05, imm16);
    auto add(uint imm32) => emit!(0, NA)(0x05, imm32);
    auto add(ulong imm32) => emit!(0, NA)(0x05, cast(long)imm32);

    auto add(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(0, NA)(0x80, dst, imm8);
    auto add(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(0, NA)(0x81, dst, imm16);
    auto add(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(0, NA)(0x81, dst, imm32);
    auto add(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(0, NA)(0x81, dst, imm32);
    auto add(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(0, NA)(0x83, dst, imm8);
    auto add(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(0, NA)(0x83, dst, imm8);
    auto add(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(0, NA)(0x83, dst, imm8);

    auto add(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x00, dst, src);
    auto add(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x01, dst, src);
    auto add(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x01, dst, src);
    auto add(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x01, dst, src);

    auto add(Reg!8 dst, Address!8 src) => emit!(0, NA)(0x02, dst, src);
    auto add(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x03, dst, src);
    auto add(Reg!32 dst, Address!32 src) => emit!(0, NA)(0x03, dst, src);
    auto add(Reg!64 dst, Address!64 src) => emit!(0, NA)(0x03, dst, src);

    auto adc(ubyte imm8) => emit!(0, NA)(0x14, imm8);
    auto adc(ushort imm16) => emit!(0, NA)(0x15, imm16);
    auto adc(uint imm32) => emit!(0, NA)(0x15, imm32);
    auto adc(ulong imm32) => emit!(0, NA)(0x15, cast(long)imm32);

    auto adc(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(2, NA)(0x80, dst, imm8);
    auto adc(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(2, NA)(0x81, dst, imm16);
    auto adc(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(2, NA)(0x81, dst, imm32);
    auto adc(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(2, NA)(0x81, dst, imm32);
    auto adc(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(2, NA)(0x83, dst, imm8);
    auto adc(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(2, NA)(0x83, dst, imm8);
    auto adc(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(2, NA)(0x83, dst, imm8);

    auto adc(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x10, dst, src);
    auto adc(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x11, dst, src);
    auto adc(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x11, dst, src);
    auto adc(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x11, dst, src);

    auto adc(Reg!8 dst, Address!8 src) => emit!(0, NA)(0x12, dst, src);
    auto adc(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x13, dst, src);
    auto adc(Reg!32 dst, Address!32 src) => emit!(0, NA)(0x13, dst, src);
    auto adc(Reg!64 dst, Address!64 src) => emit!(0, NA)(0x13, dst, src);

    auto adcx(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0F, 0x38, 0xF6, dst, src);
    auto adcx(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0F, 0x38, 0xF6, dst, src);

    auto adox(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0xF3, 0x0F, 0x38, 0xF6, dst, src);
    auto adox(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0xF3, 0x0F, 0x38, 0xF6, dst, src);

    auto and(ubyte imm8) => emit!(0, NA)(0x24, imm8);
    auto and(ushort imm16) => emit!(0, NA)(0x25, imm16);
    auto and(uint imm32) => emit!(0, NA)(0x25, imm32);
    auto and(ulong imm32) => emit!(0, NA)(0x25, cast(long)imm32);

    auto and(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(4, NA)(0x80, dst, imm8);
    auto and(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(4, NA)(0x81, dst, imm16);
    auto and(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(4, NA)(0x81, dst, imm32);
    auto and(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(4, NA)(0x81, dst, imm32);
    auto and(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(4, NA)(0x83, dst, imm8);
    auto and(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(4, NA)(0x83, dst, imm8);
    auto and(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(4, NA)(0x83, dst, imm8);

    auto and(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x20, dst, src);
    auto and(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x21, dst, src);
    auto and(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x21, dst, src);
    auto and(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x21, dst, src);

    auto and(Reg!8 dst, Address!8 src) => emit!(0, NA)(0x22, dst, src);
    auto and(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x23, dst, src);
    auto and(Reg!32 dst, Address!32 src) => emit!(0, NA)(0x23, dst, src);
    auto and(Reg!64 dst, Address!64 src) => emit!(0, NA)(0x23, dst, src);

    auto arpl(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x63, dst, src);

    auto bndcl(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0xf3, 0x0f, 0x1a, dst, src);
    auto bndcl(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0xf3, 0x0f, 0x1a, dst, src);

    auto bndcu(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0xf2, 0x0f, 0x1a, dst, src);
    auto bndcu(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0xf2, 0x0f, 0x1a, dst, src);

    auto bndcn(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0xf2, 0x0f, 0x1b, dst, src);
    auto bndcn(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0xf2, 0x0f, 0x1b, dst, src);

    auto bndldx(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NP)(0x0f, 0x1a, dst, src);
    auto bndstx(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NP)(0x0f, 0x1b, dst, src);

    auto bndmk(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0xf3, 0x0f, 0x1b, dst, src);
    auto bndmk(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0xf3, 0x0f, 0x1b, dst, src);

    auto bndmov(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x1a, dst, src);
    auto bndmov(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x1a, dst, src);
    auto bndmov(Address!32 dst, Reg!32 src) => emit!(0, NA)(0x0f, 0x1b, dst, src);
    auto bndmov(Address!64 dst, Reg!32 src) => emit!(0, NA)(0x0f, 0x1b, dst, src);

    auto bound(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x62, dst, src);
    auto bound(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x62, dst, src);

    auto bsf(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xbc, dst, src);
    auto bsf(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xbc, dst, src);
    auto bsf(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xbc, dst, src);

    auto bsr(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xbd, dst, src);
    auto bsr(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xbd, dst, src);
    auto bsr(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xbd, dst, src);

    auto bswap(Reg!32 dst) => emit!(0, NA)(0x0f, 0xc8, encodable(dst));
    auto bswap(Reg!64 dst) => emit!(0, NA)(0x0f, 0xc8, encodable(dst));

    auto bt(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xa3, dst, src); 
    auto bt(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xa3, dst, src); 
    auto bt(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xa3, dst, src); 
    auto bt(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(4, NA)(0x0f, 0xba, dst, imm8); 
    auto bt(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(4, NA)(0x0f, 0xba, dst, imm8); 
    auto bt(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(4, NA)(0x0f, 0xba, dst, imm8); 

    auto btc(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xbb, dst, src); 
    auto btc(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xbb, dst, src); 
    auto btc(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xbb, dst, src); 
    auto btc(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(7, NA)(0x0f, 0xba, dst, imm8); 
    auto btc(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(7, NA)(0x0f, 0xba, dst, imm8); 
    auto btc(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(7, NA)(0x0f, 0xba, dst, imm8); 

    auto btr(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xb3, dst, src); 
    auto btr(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xb3, dst, src); 
    auto btr(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xb3, dst, src); 
    auto btr(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(6, NA)(0x0f, 0xba, dst, imm8); 
    auto btr(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(6, NA)(0x0f, 0xba, dst, imm8); 
    auto btr(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(6, NA)(0x0f, 0xba, dst, imm8); 

    auto bts(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xab, dst, src); 
    auto bts(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xab, dst, src); 
    auto bts(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xab, dst, src); 
    auto bts(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(5, NA)(0x0f, 0xba, dst, imm8); 
    auto bts(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(5, NA)(0x0f, 0xba, dst, imm8); 
    auto bts(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(5, NA)(0x0f, 0xba, dst, imm8);

    auto cmp(ubyte imm8) => emit!(0, NA)(0x3c, imm8);
    auto cmp(ushort imm16) => emit!(0, NA)(0x3d, imm16);
    auto cmp(uint imm32) => emit!(0, NA)(0x3d, imm32);
    auto cmp(ulong imm32) => emit!(0, NA)(0x3d, cast(long)imm32);

    auto cmp(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(7, NA)(0x80, dst, imm8);
    auto cmp(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(7, NA)(0x81, dst, imm16);
    auto cmp(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(7, NA)(0x81, dst, imm32);
    auto cmp(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(7, NA)(0x81, dst, imm32);
    auto cmp(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(7, NA)(0x83, dst, imm8); 
    auto cmp(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(7, NA)(0x83, dst, imm8); 
    auto cmp(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(7, NA)(0x83, dst, imm8); 

    auto cmp(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x38, dst, src);
    auto cmp(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x39, dst, src);
    auto cmp(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x39, dst, src);
    auto cmp(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x39, dst, src);

    auto cmp(Reg!8 dst, Address!8 src) => emit!(0, NA)(0x3a, dst, src);
    auto cmp(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x3b, dst, src);
    auto cmp(Reg!32 dst, Address!32 src) => emit!(0, NA)(0x3b, dst, src);
    auto cmp(Reg!64 dst, Address!64 src) => emit!(0, NA)(0x3b, dst, src);

    auto int3() => emit!(0, NA)(0xcc);
    auto _int(ubyte imm8) => emit!(0, NA)(0xcd, imm8);
    auto into() => emit!(0, NA)(0xce);
    auto int1() => emit!(0, NA)(0xf1);
    auto ud0(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xff, dst, src);
    auto ud1(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xb9, dst, src);
    auto ud2() => emit!(0, NA)(0x0f, 0x0b);
    auto icebp() => emit!(0, NA)(0xf1);
    
    auto iret() => emit!(0, NA)(0xcf);
    auto iretd() => emit!(0, NA)(0xcf);
    auto iretq() => emit!(0, NA)(0xcf);

    auto invd() => emit!(0, NA)(0x0f, 0x08);
    auto invlpg(T)(T dst) if (isRM!(T, 64)) => emit!(7, NA)(0x0f, 0x01, dst);
    auto invpcid(Reg!32 dst, Address!128 src) => emit!(0, NA)(0x0f, 0x38, 0x82, dst, src);
    auto invpcid(Reg!64 dst, Address!128 src) => emit!(0, NA)(0x0f, 0x38, 0x82, dst, src);

    auto inc(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0xfe, dst);
    static if (X64)
    auto inc(T)(T dst) if (isRM!(T, 16)) => emit!(0, NA)(0xff, dst);
    static if (!X64)
    auto inc(Address!16 dst) => emit!(0, NA)(0xff, dst);
    static if (X64)
    auto inc(T)(T dst) if (isRM!(T, 32)) => emit!(0, NA)(0xff, dst);
    static if (!X64)
    auto inc(Address!32 dst) => emit!(0, NA)(0xff, dst);
    auto inc(T)(T dst) if (isRM!(T, 64)) => emit!(0, NA)(0xff, dst);

    static if (!X64)
    auto inc(Reg!16 dst) => emit!(0, NA)(0x40, encodable(dst));
    static if (!X64)
    auto inc(Reg!32 dst) => emit!(0, NA)(0x40, encodable(dst));

    auto dec(T)(T dst) if (isRM!(T, 8)) => emit!(1, NA)(0xfe, dst);
    static if (X64)
    auto dec(T)(T dst) if (isRM!(T, 16)) => emit!(1, NA)(0xff, dst);
    static if (!X64)
    auto dec(Address!16 dst) => emit!(1, NA)(0xff, dst);
    static if (X64)
    auto dec(T)(T dst) if (isRM!(T, 32)) => emit!(1, NA)(0xff, dst);
    static if (!X64)
    auto dec(Address!32 dst) => emit!(1, NA)(0xff, dst);
    auto dec(T)(T dst) if (isRM!(T, 64)) => emit!(1, NA)(0xff, dst);

    static if (!X64)
    auto dec(Reg!16 dst) => emit!(0, NA)(0x48, encodable(dst));
    static if (!X64)
    auto dec(Reg!32 dst) => emit!(0, NA)(0x48, encodable(dst));

    auto hlt() => emit!(0, NA)(0xf4);
    auto pause() => emit!(0, NA)(0xf3, 0x90);
    auto tpause(Reg!32 dst) => emit!(6, NA)(0x0f, 0xae, dst);
    auto monitor() => emit!(0, NA)(0x0f, 0x01, 0xc8);
    auto mwait() => emit!(0, NA)(0x0f, 0x01, 0xc9);
    auto swapgs() => emit!(0, NA)(0x0f, 0x01, 0xf8);
    auto serialize() => emit!(0, NA)(0x0f, 0x01, 0xe8);
    
    auto lock(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf0~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    auto xacquire(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    auto xacquire_lock(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~0xf0~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    auto xrelease(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    auto wait() => emit!(0, NA)(0x9b);
    auto fwait() => emit!(0, NA)(0x9b);

    auto testui() => emit!(0, NA)(0xf3, 0x0f, 0x01, 0xed);
    auto umwait(Reg!32 dst) => emit!(6, NA)(0xf2, 0x0f, 0xae, dst);
    auto umonitor(Reg!16 dst) => emit!(6, NA)(0xf3, 0x0f, 0xae, dst);
    auto umonitor(Reg!32 dst) => emit!(6, NA)(0xf3, 0x0f, 0xae, dst);
    auto umonitor(Reg!64 dst) => emit!(6, NA)(0xf3, 0x0f, 0xae, dst);
    auto uiret() => emit!(0, NA)(0xf3, 0x0f, 0x01, 0xec);
    auto senduipi(T)(T dst) if (isRM!(T, 8) || isRM!(T, 16) || isRM!(T, 32) || isRM!(T, 64)) => emit!(6, NA)(0xf3, 0x0f, 0xc7, dst);

    auto sahf() => emit!(0, NA)(0x9e);
    auto lahf() => emit!(0, NA)(0x9f);

    auto lgdt(T)(T dst) if (isRM!(T, 32)) => emit!(2, NA)(0x0f, 0x01, dst);
    auto lgdt(T)(T dst) if (isRM!(T, 64)) => emit!(2, NA)(0x0f, 0x01, dst);
    auto sgdt(T)(T dst) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x01, dst);

    auto lldt(T)(T dst) if (isRM!(T, 16)) => emit!(2, NA)(0x0f, 0x00, dst);
    auto sldt(T)(T dst) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x00, dst);
    
    auto lidt(T)(T dst) if (isRM!(T, 32)) => emit!(3, NA)(0x0f, 0x01, dst);
    auto lidt(T)(T dst) if (isRM!(T, 64)) => emit!(3, NA)(0x0f, 0x01, dst);
    auto sidt(T)(T dst) if (isRM!(T, 64)) => emit!(1, NA)(0x0f, 0x01, dst);

    auto lmsw(T)(T dst) if (isRM!(T, 16)) => emit!(6, NA)(0x0f, 0x01, dst);
    auto smsw(T)(T dst) if (isRM!(T, 16)) => emit!(4, NA)(0x0f, 0x01, dst);
    auto smsw(T)(T dst) if (isRM!(T, 32)) => emit!(4, NA)(0x0f, 0x01, dst);
    auto smsw(T)(T dst) if (isRM!(T, 64)) => emit!(4, NA)(0x0f, 0x01, dst);

    auto sysenter() => emit!(0, NA)(0x0f, 0x34);
    auto sysexitc() => emit!(0, NA)(0x0f, 0x35);
    auto sysexit() => emit!(0, NA)(0x0f, 0x35);
    auto sysretc() => emit!(0, NA)(0x0f, 0x07);
    auto sysret() => emit!(0, NA)(0x0f, 0x07);
    auto syscall() => emit!(0, NA)(0x0f, 0x05);
    auto rsm() => emit!(0, NA)(0x0f, 0xaa);

    auto leave() => emit!(0, NA)(0xc9);
    auto enter(ushort imm16) => emit!(0, NA)(0xc8, imm16, 0x00);
    auto enter(ushort imm16, ubyte imm8) => emit!(0, NA)(0xc8, imm16, imm8);
    
    auto lea(T)(Reg!16 dst, Address!16) => emit!(0, NA)(0x8d, dst, src);
    auto lea(T)(Reg!32 dst, Address!32) => emit!(0, NA)(0x8d, dst, src);
    auto lea(T)(Reg!64 dst, Address!64) => emit!(0, NA)(0x8d, dst, src);

    auto lds(T)(Reg!16 dst, Address!16) => emit!(0, NA)(0xc5, dst, src);
    auto lds(T)(Reg!32 dst, Address!32) => emit!(0, NA)(0xc5, dst, src);

    auto lss(T)(Reg!16 dst, Address!16) => emit!(0, NA)(0x0f, 0xb2, dst, src);
    auto lss(T)(Reg!32 dst, Address!32) => emit!(0, NA)(0x0f, 0xb2, dst, src);
    auto lss(T)(Reg!64 dst, Address!64) => emit!(0, NA)(0x0f, 0xb2, dst, src);

    auto les(T)(Reg!16 dst, Address!16) => emit!(0, NA)(0xc4, dst, src);
    auto les(T)(Reg!32 dst, Address!32) => emit!(0, NA)(0xc4, dst, src);

    auto lfs(T)(Reg!16 dst, Address!16) => emit!(0, NA)(0x0f, 0xb4, dst, src);
    auto lfs(T)(Reg!32 dst, Address!32) => emit!(0, NA)(0x0f, 0xb4, dst, src);
    auto lfs(T)(Reg!64 dst, Address!64) => emit!(0, NA)(0x0f, 0xb4, dst, src);

    auto lgs(T)(Reg!16 dst, Address!16) => emit!(0, NA)(0x0f, 0xb5, dst, src);
    auto lgs(T)(Reg!32 dst, Address!32) => emit!(0, NA)(0x0f, 0xb5, dst, src);
    auto lgs(T)(Reg!64 dst, Address!64) => emit!(0, NA)(0x0f, 0xb5, dst, src);

    auto lsl(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x03, dst, src);
    auto lsl(Reg!32 dst, Reg!32 src) => emit!(0, NA)(0x0f, 0x03, dst, src);
    auto lsl(Reg!64 dst, Reg!32 src) => emit!(0, NA)(0x0f, 0x03, dst, src);
    auto lsl(Reg!32 dst, Address!16 src) => emit!(0, NA)(0x0f, 0x03, dst, src);
    auto lsl(Reg!64 dst, Address!16 src) => emit!(0, NA)(0x0f, 0x03, dst, src);

    auto ltr(T)(T dst) if (isRM!(T, 16)) => emit!(3, NA)(0x0f, 0x00, dst);
    auto str(T)(T dst) if (isRM!(T, 16)) => emit!(1, NA)(0x0f, 0x00, dst);

    auto neg(T)(T dst) if (isRM!(T, 8)) => emit!(3, NA)(0xf6, dst);
    auto neg(T)(T dst) if (isRM!(T, 16)) => emit!(3, NA)(0xf7, dst);
    auto neg(T)(T dst) if (isRM!(T, 32)) => emit!(3, NA)(0xf7, dst);
    auto neg(T)(T dst) if (isRM!(T, 64)) => emit!(3, NA)(0xf7, dst);

    auto nop() => emit!(0, NA)(0x90);
    auto nop(T)(T dst) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x1f, dst);
    auto nop(T)(T dst) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x1f, dst);

    auto not(T)(T dst) if (isRM!(T, 8)) => emit!(2, NA)(0xf6, dst);
    auto not(T)(T dst) if (isRM!(T, 16)) => emit!(2, NA)(0xf7, dst);
    auto not(T)(T dst) if (isRM!(T, 32)) => emit!(2, NA)(0xf7, dst);
    auto not(T)(T dst) if (isRM!(T, 64)) => emit!(2, NA)(0xf7, dst);

    auto ret() => emit!(0, NA)(0xc3);
    auto ret(ushort imm16) => emit!(0, NA)(0xc2, imm16);
    auto retf() => emit!(0, NA)(0xcb);
    auto retf(ushort imm16) => emit!(0, NA)(0xca, imm16);

    auto clac() => emit!(0, NA)(0x0f, 0x01, 0xca);
    auto clc() => emit!(0, NA)(0xf8);
    auto cld() => emit!(0, NA)(0xfc);
    auto cli() => emit!(0, NA)(0xfa);
    auto clui() => emit!(0, NA)(0xf3, 0x0f, 0x01, 0xee);
    auto clts() => emit!(0, NA)(0x0f, 0x06);

    auto stac() => emit!(0, NA)(0x0f, 0x01, 0xcb);
    auto stc() => emit!(0, NA)(0xf9);
    auto std() => emit!(0, NA)(0xfd);
    auto sti() => emit!(0, NA)(0xfb);
    auto stui() => emit!(0, NA)(0xf3, 0x0f, 0x01, 0xef);

    auto cmc() => emit!(0, NA)(0xf5);

    auto sub(ubyte imm8) => emit!(0, NA)(0x2c, imm8);
    auto sub(ushort imm16) => emit!(0, NA)(0x2d, imm16);
    auto sub(uint imm32) => emit!(0, NA)(0x2d, imm32);
    auto sub(ulong imm32) => emit!(0, NA)(0x2d, cast(long)imm32);

    auto sub(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(5, NA)(0x80, dst, imm8);
    auto sub(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(5, NA)(0x81, dst, imm16);
    auto sub(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(5, NA)(0x81, dst, imm32);
    auto sub(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(5, NA)(0x81, dst, imm32);
    auto sub(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(5, NA)(0x83, dst, imm8);
    auto sub(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(5, NA)(0x83, dst, imm8);
    auto sub(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(5, NA)(0x83, dst, imm8);

    auto sub(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x28, dst, src);
    auto sub(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x29, dst, src);
    auto sub(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x29, dst, src);
    auto sub(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x29, dst, src);

    auto sub(Reg!8 dst, Address!8 src) => emit!(0, NA)(0x2a, dst, src);
    auto sub(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x2b, dst, src);
    auto sub(Reg!32 dst, Address!32 src) => emit!(0, NA)(0x2b, dst, src);
    auto sub(Reg!64 dst, Address!64 src) => emit!(0, NA)(0x2b, dst, src);

    auto sbb(ubyte imm8) => emit!(0, NA)(0x1c, imm8);
    auto sbb(ushort imm16) => emit!(0, NA)(0x1d, imm16);
    auto sbb(uint imm32) => emit!(0, NA)(0x1d, imm32);
    auto sbb(ulong imm32) => emit!(0, NA)(0x1d, cast(long)imm32);

    auto sbb(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(3, NA)(0x80, dst, imm8);
    auto sbb(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(3, NA)(0x81, dst, imm16);
    auto sbb(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(3, NA)(0x81, dst, imm32);
    auto sbb(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(3, NA)(0x81, dst, imm32);
    auto sbb(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(3, NA)(0x83, dst, imm8);
    auto sbb(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(3, NA)(0x83, dst, imm8);
    auto sbb(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(3, NA)(0x83, dst, imm8);

    auto sbb(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x18, dst, src);
    auto sbb(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x19, dst, src);
    auto sbb(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x19, dst, src);
    auto sbb(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x19, dst, src);

    auto sbb(Reg!8 dst, Address!8 src) => emit!(0, NA)(0x1a, dst, src);
    auto sbb(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x1b, dst, src);
    auto sbb(Reg!32 dst, Address!32 src) => emit!(0, NA)(0x1b, dst, src);
    auto sbb(Reg!64 dst, Address!64 src) => emit!(0, NA)(0x1b, dst, src);

    auto xor(ubyte imm8) => emit!(0, NA)(0x34, imm8);
    auto xor(ushort imm16) => emit!(0, NA)(0x35, imm16);
    auto xor(uint imm32) => emit!(0, NA)(0x35, imm32);
    auto xor(ulong imm32) => emit!(0, NA)(0x35, cast(long)imm32);

    auto xor(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(6, NA)(0x80, dst, imm8);
    auto xor(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(6, NA)(0x81, dst, imm16);
    auto xor(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(6, NA)(0x81, dst, imm32);
    auto xor(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(6, NA)(0x81, dst, imm32);
    auto xor(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(6, NA)(0x83, dst, imm8);
    auto xor(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(6, NA)(0x83, dst, imm8);
    auto xor(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(6, NA)(0x83, dst, imm8);

    auto xor(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x30, dst, src);
    auto xor(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x31, dst, src);
    auto xor(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x31, dst, src);
    auto xor(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x31, dst, src);

    auto xor(Reg!8 dst, Address!8 src) => emit!(0, NA)(0x32, dst, src);
    auto xor(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x33, dst, src);
    auto xor(Reg!32 dst, Address!32 src) => emit!(0, NA)(0x33, dst, src);
    auto xor(Reg!64 dst, Address!64 src) => emit!(0, NA)(0x33, dst, src);

    auto or(ubyte imm8) => emit!(0, NA)(0x0c, imm8);
    auto or(ushort imm16) => emit!(0, NA)(0x0d, imm16);
    auto or(uint imm32) => emit!(0, NA)(0x0d, imm32);
    auto or(ulong imm32) => emit!(0, NA)(0x0d, cast(long)imm32);

    auto or(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(1, NA)(0x80, dst, imm8);
    auto or(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(1, NA)(0x81, dst, imm16);
    auto or(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(1, NA)(0x81, dst, imm32);
    auto or(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(1, NA)(0x81, dst, imm32);
    auto or(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => emit!(1, NA)(0x83, dst, imm8);
    auto or(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => emit!(1, NA)(0x83, dst, imm8);
    auto or(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => emit!(1, NA)(0x83, dst, imm8);

    auto or(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x8, dst, src);
    auto or(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x9, dst, src);
    auto or(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x9, dst, src);
    auto or(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x9, dst, src);

    auto or(Reg!8 dst, Address!8 src) => emit!(0, NA)(0xa, dst, src);
    auto or(Reg!16 dst, Address!16 src) => emit!(0, NA)(0xb, dst, src);
    auto or(Reg!32 dst, Address!32 src) => emit!(0, NA)(0xb, dst, src);
    auto or(Reg!64 dst, Address!64 src) => emit!(0, NA)(0xb, dst, src);

    auto sal(T)(T dst) if (isRM!(T, 8)) => emit!(4, NA)(0xd2, dst, cl);
    auto sal(T)(T dst, ubyte imm8) if (isRM!(T, 8))
    {
        if (imm8 == 1)
            return emit!(4, NA)(0xd0, dst);
        else
            return emit!(4, NA)(0xc0, dst, imm8);
    }
    auto sal(T)(T dst) if (isRM!(T, 16)) => emit!(4, NA)(0xd3, dst, cl);
    auto sal(T)(T dst, ubyte imm8) if (isRM!(T, 16))
    {
        if (imm8 == 1)
            return emit!(4, NA)(0xd1, dst);
        else
            return emit!(4, NA)(0xc1, dst, imm8);
    }
    auto sal(T)(T dst) if (isRM!(T, 32)) => emit!(4, NA)(0xd3, dst, cl);
    auto sal(T)(T dst, ubyte imm8) if (isRM!(T, 32))
    {
        if (imm8 == 1)
            return emit!(4, NA)(0xd1, dst);
        else
            return emit!(4, NA)(0xc1, dst, imm8);
    }
    auto sal(T)(T dst) if (isRM!(T, 64)) => emit!(4, NA)(0xd3, dst, cl);
    auto sal(T)(T dst, ubyte imm8) if (isRM!(T, 64))
    {
        if (imm8 == 1)
            return emit!(4, NA)(0xd1, dst);
        else
            return emit!(4, NA)(0xc1, dst, imm8);
    }

    auto sar(T)(T dst) if (isRM!(T, 8)) => emit!(7, NA)(0xd2, dst, cl);
    auto sar(T)(T dst, ubyte imm8) if (isRM!(T, 8))
    {
        if (imm8 == 1)
            return emit!(7, NA)(0xd0, dst);
        else
            return emit!(7, NA)(0xc0, dst, imm8);
    }
    auto sar(T)(T dst) if (isRM!(T, 16)) => emit!(7, NA)(0xd3, dst, cl);
    auto sar(T)(T dst, ubyte imm8) if (isRM!(T, 16))
    {
        if (imm8 == 1)
            return emit!(7, NA)(0xd1, dst);
        else
            return emit!(7, NA)(0xc1, dst, imm8);
    }
    auto sar(T)(T dst) if (isRM!(T, 32)) => emit!(7, NA)(0xd3, dst, cl);
    auto sar(T)(T dst, ubyte imm8) if (isRM!(T, 32))
    {
        if (imm8 == 1)
            return emit!(7, NA)(0xd1, dst);
        else
            return emit!(7, NA)(0xc1, dst, imm8);
    }
    auto sar(T)(T dst) if (isRM!(T, 64)) => emit!(7, NA)(0xd3, dst, cl);
    auto sar(T)(T dst, ubyte imm8) if (isRM!(T, 64))
    {
        if (imm8 == 1)
            return emit!(7, NA)(0xd1, dst);
        else
            return emit!(7, NA)(0xc1, dst, imm8);
    }

    auto shl(T)(T dst) if (isRM!(T, 8)) => sal(dst);
    auto shl(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => sal(dst, imm8);
    auto shl(T)(T dst) if (isRM!(T, 16)) => sal(dst);
    auto shl(T)(T dst, ubyte imm8) if (isRM!(T, 16)) => sal(dst, imm8);
    auto shl(T)(T dst) if (isRM!(T, 32)) => sal(dst);
    auto shl(T)(T dst, ubyte imm8) if (isRM!(T, 32)) => sal(dst, imm8);
    auto shl(T)(T dst) if (isRM!(T, 64)) => sal(dst);
    auto shl(T)(T dst, ubyte imm8) if (isRM!(T, 64)) => sal(dst, imm8);

    auto shr(T)(T dst) if (isRM!(T, 8)) => emit!(5, NA)(0xd2, dst, cl);
    auto shr(T)(T dst, ubyte imm8) if (isRM!(T, 8))
    {
        if (imm8 == 1)
            return emit!(5, NA)(0xd0, dst);
        else
            return emit!(5, NA)(0xc0, dst, imm8);
    }
    auto shr(T)(T dst) if (isRM!(T, 16)) => emit!(5, NA)(0xd3, dst, cl);
    auto shr(T)(T dst, ubyte imm8) if (isRM!(T, 16))
    {
        if (imm8 == 1)
            return emit!(5, NA)(0xd1, dst);
        else
            return emit!(5, NA)(0xc1, dst, imm8);
    }
    auto shr(T)(T dst) if (isRM!(T, 32)) => emit!(5, NA)(0xd3, dst, cl);
    auto shr(T)(T dst, ubyte imm8) if (isRM!(T, 32))
    {
        if (imm8 == 1)
            return emit!(5, NA)(0xd1, dst);
        else
            return emit!(5, NA)(0xc1, dst, imm8);
    }
    auto shr(T)(T dst) if (isRM!(T, 64)) => emit!(5, NA)(0xd3, dst, cl);
    auto shr(T)(T dst, ubyte imm8) if (isRM!(T, 64))
    {
        if (imm8 == 1)
            return emit!(5, NA)(0xd1, dst);
        else
            return emit!(5, NA)(0xc1, dst, imm8);
    }

    auto rcl(T)(T dst) if (isRM!(T, 8)) => emit!(2, NA)(0xd2, dst, cl);
    auto rcl(T)(T dst, ubyte imm8) if (isRM!(T, 8))
    {
        if (imm8 == 1)
            return emit!(2, NA)(0xd0, dst);
        else
            return emit!(2, NA)(0xc0, dst, imm8);
    }
    auto rcl(T)(T dst) if (isRM!(T, 16)) => emit!(2, NA)(0xd3, dst, cl);
    auto rcl(T)(T dst, ubyte imm8) if (isRM!(T, 16))
    {
        if (imm8 == 1)
            return emit!(2, NA)(0xd1, dst);
        else
            return emit!(2, NA)(0xc1, dst, imm8);
    }
    auto rcl(T)(T dst) if (isRM!(T, 32)) => emit!(2, NA)(0xd3, dst, cl);
    auto rcl(T)(T dst, ubyte imm8) if (isRM!(T, 32))
    {
        if (imm8 == 1)
            return emit!(2, NA)(0xd1, dst);
        else
            return emit!(2, NA)(0xc1, dst, imm8);
    }
    auto rcl(T)(T dst) if (isRM!(T, 64)) => emit!(2, NA)(0xd3, dst, cl);
    auto rcl(T)(T dst, ubyte imm8) if (isRM!(T, 64))
    {
        if (imm8 == 1)
            return emit!(2, NA)(0xd1, dst);
        else
            return emit!(2, NA)(0xc1, dst, imm8);
    }

    auto rcr(T)(T dst) if (isRM!(T, 8)) => emit!(3, NA)(0xd2, dst, cl);
    auto rcr(T)(T dst, ubyte imm8) if (isRM!(T, 8))
    {
        if (imm8 == 1)
            return emit!(3, NA)(0xd0, dst);
        else
            return emit!(3, NA)(0xc0, dst, imm8);
    }
    auto rcr(T)(T dst) if (isRM!(T, 16)) => emit!(3, NA)(0xd3, dst, cl);
    auto rcr(T)(T dst, ubyte imm8) if (isRM!(T, 16))
    {
        if (imm8 == 1)
            return emit!(3, NA)(0xd1, dst);
        else
            return emit!(3, NA)(0xc1, dst, imm8);
    }
    auto rcr(T)(T dst) if (isRM!(T, 32)) => emit!(3, NA)(0xd3, dst, cl);
    auto rcr(T)(T dst, ubyte imm8) if (isRM!(T, 32))
    {
        if (imm8 == 1)
            return emit!(3, NA)(0xd1, dst);
        else
            return emit!(3, NA)(0xc1, dst, imm8);
    }
    auto rcr(T)(T dst) if (isRM!(T, 64)) => emit!(3, NA)(0xd3, dst, cl);
    auto rcr(T)(T dst, ubyte imm8) if (isRM!(T, 64))
    {
        if (imm8 == 1)
            return emit!(3, NA)(0xd1, dst);
        else
            return emit!(3, NA)(0xc1, dst, imm8);
    }

    auto rol(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0xd2, dst, cl);
    auto rol(T)(T dst, ubyte imm8) if (isRM!(T, 8))
    {
        if (imm8 == 1)
            return emit!(0, NA)(0xd0, dst);
        else
            return emit!(0, NA)(0xc0, dst, imm8);
    }
    auto rol(T)(T dst) if (isRM!(T, 16)) => emit!(0, NA)(0xd3, dst, cl);
    auto rol(T)(T dst, ubyte imm8) if (isRM!(T, 16))
    {
        if (imm8 == 1)
            return emit!(0, NA)(0xd1, dst);
        else
            return emit!(0, NA)(0xc1, dst, imm8);
    }
    auto rol(T)(T dst) if (isRM!(T, 32)) => emit!(0, NA)(0xd3, dst, cl);
    auto rol(T)(T dst, ubyte imm8) if (isRM!(T, 32))
    {
        if (imm8 == 1)
            return emit!(0, NA)(0xd1, dst);
        else
            return emit!(0, NA)(0xc1, dst, imm8);
    }
    auto rol(T)(T dst) if (isRM!(T, 64)) => emit!(0, NA)(0xd3, dst, cl);
    auto rol(T)(T dst, ubyte imm8) if (isRM!(T, 64))
    {
        if (imm8 == 1)
            return emit!(0, NA)(0xd1, dst);
        else
            return emit!(0, NA)(0xc1, dst, imm8);
    }

    auto ror(T)(T dst) if (isRM!(T, 8)) => emit!(1, NA)(0xd2, dst, cl);
    auto ror(T)(T dst, ubyte imm8) if (isRM!(T, 8))
    {
        if (imm8 == 1)
            return emit!(1, NA)(0xd0, dst);
        else
            return emit!(1, NA)(0xc0, dst, imm8);
    }
    auto ror(T)(T dst) if (isRM!(T, 16)) => emit!(1, NA)(0xd3, dst, cl);
    auto ror(T)(T dst, ubyte imm8) if (isRM!(T, 16))
    {
        if (imm8 == 1)
            return emit!(1, NA)(0xd1, dst);
        else
            return emit!(1, NA)(0xc1, dst, imm8);
    }
    auto ror(T)(T dst) if (isRM!(T, 32)) => emit!(1, NA)(0xd3, dst, cl);
    auto ror(T)(T dst, ubyte imm8) if (isRM!(T, 32))
    {
        if (imm8 == 1)
            return emit!(1, NA)(0xd1, dst);
        else
            return emit!(1, NA)(0xc1, dst, imm8);
    }
    auto ror(T)(T dst) if (isRM!(T, 64)) => emit!(1, NA)(0xd3, dst, cl);
    auto ror(T)(T dst, ubyte imm8) if (isRM!(T, 64))
    {
        if (imm8 == 1)
            return emit!(1, NA)(0xd1, dst);
        else
            return emit!(1, NA)(0xc1, dst, imm8);
    }

    auto verr(T)(T dst) if (isRM!(T, 16)) => emit!(4, NA)(0xf0, 0x00, dst);
    auto verw(T)(T dst) if (isRM!(T, 16)) => emit!(5, NA)(0xf0, 0x00, dst);

    auto test(ubyte imm8) => emit!(0, NA)(0xa8, imm8);
    auto test(ushort imm16) => emit!(0, NA)(0xa9, imm16);
    auto test(uint imm32) => emit!(0, NA)(0xa9, imm32);
    auto test(ulong imm32) => emit!(0, NA)(0xa9, cast(long)imm32);

    auto test(T)(T dst, ubyte imm8) if (isRM!(T, 8)) => emit!(0, NA)(0xf6, dst, imm8);
    auto test(T)(T dst, ushort imm16) if (isRM!(T, 16)) => emit!(0, NA)(0xf7, dst, imm16);
    auto test(T)(T dst, uint imm32) if (isRM!(T, 32)) => emit!(0, NA)(0xf7, dst, imm32);
    auto test(T)(T dst, uint imm32) if (isRM!(T, 64)) => emit!(0, NA)(0xf7, dst, cast(long)imm32);

    auto test(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x84, dst, src);
    auto test(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x85, dst, src);
    auto test(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x85, dst, src);
    auto test(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x85, dst, src);

    auto pop(T)(T dst) if (isRM!(T, 16)) => emit!(0, NA)(0x8f, dst);
    static if (!X64)
    auto pop(T)(T dst) if (isRM!(T, 32)) => emit!(0, NA)(0x8f, dst);
    static if (X64)
    auto pop(T)(T dst) if (isRM!(T, 64)) => emit!(0, NA)(0x8f, dst);

    auto pop(Reg!16 dst) => emit!(0, NA)(0x58, encodable(dst));
    static if (!X64)
    auto pop(Reg!32 dst) => emit!(0, NA)(0x58, encodable(dst));
    static if (X64)
    auto pop(Reg!64 dst) => emit!(0, NA)(0x58, encodable(dst));

    auto popds() => emit!(0, NA)(0x1f);
    auto popes() => emit!(0, NA)(0x07);
    auto popss() => emit!(0, NA)(0x17);
    auto popfs() => emit!(0, NA)(0x0f, 0xa1);
    auto popgs() => emit!(0, NA)(0x0f, 0xa9); 

    auto popa() => emit!(0, NA)(0x61);
    auto popad() => emit!(0, NA)(0x61);

    auto popf() => emit!(0, NA)(0x9d);
    auto popfd() => emit!(0, NA)(0x9d);
    auto popfq() => emit!(0, NA)(0x9d);
    
    auto popcnt(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0xf3, 0x0f, 0xb8, dst, src);
    auto popcnt(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0xf3, 0x0f, 0xb8, dst, src);
    auto popcnt(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0xf3, 0x0f, 0xb8, dst, src);

    auto push(T)(T dst) if (isRM!(T, 16)) => emit!(6, NA)(0xff, dst);
    static if (!X64)
    auto push(T)(T dst) if (isRM!(T, 32)) => emit!(6, NA)(0xff, dst);
    static if (X64)
    auto push(T)(T dst) if (isRM!(T, 64)) => emit!(6, NA)(0xff, dst);

    auto push(Reg!16 dst) => emit!(0, NA)(0x50, encodable(dst));
    static if (!X64)
    auto push(Reg!32 dst) => emit!(0, NA)(0x50, encodable(dst));
    static if (X64)
    auto push(Reg!64 dst) => emit!(0, NA)(0x50, encodable(dst));

    auto push(ubyte imm8) => emit!(0, NA)(0x6a, imm8);
    auto push(ushort imm16) => emit!(0, NA)(0x68, imm16);
    auto push(uint imm32) => emit!(0, NA)(0x68, imm32);

    auto pushcs() => emit!(0, NA)(0x0e);
    auto pushss() => emit!(0, NA)(0x16);
    auto pushds() => emit!(0, NA)(0x1e);
    auto pushes() => emit!(0, NA)(0x06);
    auto pushfs() => emit!(0, NA)(0x0f, 0xa0);
    auto pushgs() => emit!(0, NA)(0x0f, 0xa8); 

    auto pusha() => emit!(0, NA)(0x60);
    auto pushad() => emit!(0, NA)(0x60);

    auto pushf() => emit!(0, NA)(0x9c);
    auto pushfd() => emit!(0, NA)(0x9c);
    auto pushfq() => emit!(0, NA)(0x9c);

    auto cwd() => emit!(0, NA)(0x66, 0x99);
    auto cdq() => emit!(0, NA)(0x99);
    auto cqo() => emit!(0, NA)(0x48, 0x99);

    auto cbw() => emit!(0, NA)(0x66, 0x98);
    auto cwde() => emit!(0, NA)(0x98);
    auto cdqe() => emit!(0, NA)(0x48, 0x98);

    auto cpuid() => emit!(0, NA)(0x0f, 0xa2);
    auto hreset(ubyte imm8) => emit!(0, NA)(0xf3, 0x0f, 0x3a, 0xf0, 0xc0, imm8, eax);

    auto xabort(ubyte imm8) => emit!(0, NA)(0xc6, 0xf8, imm8);
    auto xend() => emit!(0, NA)(0x0f, 0x01, 0xd5);
    auto xbegin(ushort rel16) => emit!(0, NA)(0xc7, 0xf8, rel16);
    auto xbegin(uint rel32) => emit!(0, NA)(0xc7, 0xf8, rel32);
    auto xtest() => emit!(0, NA)(0x0f, 0x01, 0xd6);
    auto xgetbv() => emit!(0, NA)(0x0f, 0x01, 0xd0);
    auto xsetbv() => emit!(0, NA)(0x0f, 0x01, 0xd1);

    auto xadd(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0xc0, dst, src);
    auto xadd(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xc1, dst, src);
    auto xadd(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xc1, dst, src);
    auto xadd(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xc1, dst, src);

    auto xchg(Reg!16 dst) => emit!(0, NA)(90, encodable(dst));
    auto xchg(Reg!32 dst) => emit!(0, NA)(90, encodable(dst));
    auto xchg(Reg!64 dst) => emit!(0, NA)(90, encodable(dst));

    auto xchg(A, B)(A dst, B src) if (isRM!(A, 8) && isRM!(B, 8)) => emit!(0, NA)(0x86, dst, src);
    auto xchg(A, B)(A dst, B src) if (isRM!(A, 16) && isRM!(B, 16)) => emit!(0, NA)(0x87, dst, src);
    auto xchg(A, B)(A dst, B src) if (isRM!(A, 32) && isRM!(B, 32)) => emit!(0, NA)(0x87, dst, src);
    auto xchg(A, B)(A dst, B src) if (isRM!(A, 64) && isRM!(B, 64)) => emit!(0, NA)(0x87, dst, src);

    auto xlat() => emit!(0, NA)(0xd7);
    static if (!X64)
    auto xlatb() => emit!(0, NA)(0xd7);
    static if (X64)
    auto xlatb() => emit!(0, NA)(0x48, 0xd7);

    auto xresldtrk() => emit!(0, NA)(0xf2, 0x0f, 0x01, 0xe9);
    auto xsusldtrk() => emit!(0, NA)(0xf2, 0x0f, 0x01, 0xe8);

    auto xrstor(T)(T dst) if (isInstanceOf!(Address, T)) => emit!(5, NP)(0x0f, 0xae, dst);
    auto xrstors(T)(T dst) if (isInstanceOf!(Address, T)) => emit!(3, NP)(0x0f, 0xc7, dst);

    auto xsave(T)(T dst) if (isInstanceOf!(Address, T)) => emit!(4, NP)(0x0f, 0xae, dst);
    auto xsavec(T)(T dst) if (isInstanceOf!(Address, T)) => emit!(4, NP)(0x0f, 0xc7, dst);
    auto xsaveopt(T)(T dst) if (isInstanceOf!(Address, T)) => emit!(6, NP)(0x0f, 0xae, dst);
    auto xsaves(T)(T dst) if (isInstanceOf!(Address, T)) => emit!(5, NP)(0x0f, 0xc7, dst);

    auto clflush(T)(T dst) if (isRM!(T, 8)) => emit!(7, NP)(0x0f, 0xae, dst);
    auto clflushopt(T)(T dst) if (isRM!(T, 8)) => emit!(7, NA)(0x66, 0x0f, 0xae, dst);
    auto cldemote(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x1c, dst);
    auto clwd(T)(T dst) if (isRM!(T, 8)) => emit!(6, NA)(0x66, 0x0f, 0xae, dst);

    auto wbinvd() => emit!(0, NA)(0x0f, 0x09);
    auto wbnoinvd() => emit!(0, NA)(0xf3, 0x0f, 0x09);

    auto rdmsr() => emit!(0, NA)(0x0f, 0x32);
    auto wrmsr() => emit!(0, NA)(0x0f, 0x30);
    auto rdpmc() => emit!(0, NA)(0x0f, 0x33);

    auto wrpkru() => emit!(0, NA)(0x0f, 0x01, 0xef);
    auto rdpkru() => emit!(0, NA)(0x0f, 0x01, 0xee);

    auto lar(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x0f, 0x02, dst, src);
    auto lar(Reg!16 dst, Reg!16 src) => emit!(0, NA)(0x0f, 0x02, dst, src);
    auto lar(Reg!32 dst, Address!16 src) => emit!(0, NA)(0x0f, 0x02, dst, src);
    auto lar(Reg!32 dst, Reg!32 src) => emit!(0, NA)(0x0f, 0x02, dst, src);

    auto rdpid(Reg!32 dst) => emit!(7, NA)(0xf3, 0x0f, 0xc7, dst);
    auto rdpid(Reg!64 dst) => emit!(7, NA)(0xf3, 0x0f, 0xc7, dst);

    auto rdseed(Reg!16 dst) => emit!(7, NA)(0x0f, 0xc7, dst);
    auto rdseed(Reg!32 dst) => emit!(7, NA)(0x0f, 0xc7, dst);
    auto rdseed(Reg!64 dst) => emit!(7, NA)(0x0f, 0xc7, dst);

    auto rdrand(Reg!16 dst) => emit!(6, NA)(0x0f, 0xc7, dst);
    auto rdrand(Reg!32 dst) => emit!(6, NA)(0x0f, 0xc7, dst);
    auto rdrand(Reg!64 dst) => emit!(6, NA)(0x0f, 0xc7, dst);

    auto rdfsbase(Reg!32 dst) => emit!(0, NA)(0xf3, 0x0f, 0xae, dst);
    auto rdfsbase(Reg!64 dst) => emit!(0, NA)(0xf3, 0x0f, 0xae, dst);
    auto rdgsbase(Reg!32 dst) => emit!(1, NA)(0xf3, 0x0f, 0xae, dst);
    auto rdgsbase(Reg!64 dst) => emit!(1, NA)(0xf3, 0x0f, 0xae, dst);

    auto wrfsbase(Reg!32 dst) => emit!(2, NA)(0xf3, 0x0f, 0xae, dst);
    auto wrfsbase(Reg!64 dst) => emit!(2, NA)(0xf3, 0x0f, 0xae, dst);
    auto wrgsbase(Reg!32 dst) => emit!(3, NA)(0xf3, 0x0f, 0xae, dst);
    auto wrgsbase(Reg!64 dst) => emit!(3, NA)(0xf3, 0x0f, 0xae, dst);

    auto incsspd(Reg!32 dst) => emit!(5, NA)(0xf3, 0x0f, 0xae, dst);
    auto incsspq(Reg!64 dst) => emit!(5, NA)(0xf3, 0x0f, 0xae, dst);
    auto clrssbsy(Address!64 dst) => emit!(6, NA)(0xf3, 0x0f, 0xae, dst);
    auto setssbsy() => emit!(0, NA)(0xf3, 0x0f, 0x01, 0xe8);

    auto rdsspd(Reg!32 dst) => emit!(1, NA)(0xf3, 0x0f, 0x1e, dst);
    auto rdsspq(Reg!64 dst) => emit!(1, NA)(0xf3, 0x0f, 0x1e, dst);
    auto wrssd(Address!32 dst, Reg!32 src) => emit!(0, NA)(0xf3, 0x38, 0xf6, dst, src);
    auto wrssq(Address!64 dst, Reg!64 src) => emit!(0, NA)(0xf3, 0x38, 0xf6, dst, src);
    auto wrussd(Address!32 dst, Reg!32 src) => emit!(1, NA)(0x66, 0xf3, 0x38, 0xf5, dst, src);
    auto wrussq(Address!64 dst, Reg!64 src) => emit!(1, NA)(0x66, 0xf3, 0x38, 0xf5, dst, src);

    auto rstorssp(Address!64 dst) => emit!(5, NA)(0xf3, 0x0f, 0x01, dst);
    auto saveprevssp() => emit!(5, NA)(0xf3, 0x0f, 0x01, 0xae, edx);

    auto daa() => emit!(0, NA)(0x27);
    auto das() => emit!(0, NA)(0x2f);

    auto mul(T)(T dst) if (isRM!(T, 8)) => emit!(4, NA)(0xf6, dst);
    auto mul(T)(T dst) if (isRM!(T, 16)) => emit!(4, NA)(0xf7, dst);
    auto mul(T)(T dst) if (isRM!(T, 32)) => emit!(4, NA)(0xf7, dst);
    auto mul(T)(T dst) if (isRM!(T, 64)) => emit!(4, NA)(0xf7, dst);

    auto imul(T)(T dst) if (isRM!(T, 8)) => emit!(5, NA)(0xf6, dst);
    auto imul(T)(T dst) if (isRM!(T, 16)) => emit!(5, NA)(0xf7, dst);
    auto imul(T)(T dst) if (isRM!(T, 32)) => emit!(5, NA)(0xf7, dst);
    auto imul(T)(T dst) if (isRM!(T, 64)) => emit!(5, NA)(0xf7, dst);

    auto imul(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xaf, dst, src);
    auto imul(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0xaf, dst, src);
    auto imul(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0xaf, dst, src);

    auto imul(T)(Reg!16 dst, T src, ubyte imm8) if (isRM!(T, 16)) => emit!(0, NA)(0x6b, dst, src, imm8);
    auto imul(T)(Reg!32 dst, T src, ubyte imm8) if (isRM!(T, 32)) => emit!(0, NA)(0x6b, dst, src, imm8);
    auto imul(T)(Reg!64 dst, T src, ubyte imm8) if (isRM!(T, 64)) => emit!(0, NA)(0x6b, dst, src, imm8);
    auto imul(T)(Reg!16 dst, T src, ushort imm16) if (isRM!(T, 16)) => emit!(0, NA)(0x69, dst, src, imm16);
    auto imul(T)(Reg!32 dst, T src, uint imm32) if (isRM!(T, 32)) => emit!(0, NA)(0x69, dst, src, imm32);
    auto imul(T)(Reg!64 dst, T src, uint imm32) if (isRM!(T, 64)) => emit!(0, NA)(0x69, dst, src, imm32);

    auto div(T)(T dst) if (isRM!(T, 8)) => emit!(6, NA)(0xf6, dst);
    auto div(T)(T dst) if (isRM!(T, 16)) => emit!(6, NA)(0xf7, dst);
    auto div(T)(T dst) if (isRM!(T, 32)) => emit!(6, NA)(0xf7, dst);
    auto div(T)(T dst) if (isRM!(T, 64)) => emit!(6, NA)(0xf7, dst);

    auto idiv(T)(T dst) if (isRM!(T, 8)) => emit!(7, NA)(0xf6, dst);
    auto idiv(T)(T dst) if (isRM!(T, 16)) => emit!(7, NA)(0xf7, dst);
    auto idiv(T)(T dst) if (isRM!(T, 32)) => emit!(7, NA)(0xf7, dst);
    auto idiv(T)(T dst) if (isRM!(T, 64)) => emit!(7, NA)(0xf7, dst);

    auto tzcnt(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0xf3, 0x0f, 0xbc, dst, src);
    auto tzcnt(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0xf3, 0x0f, 0xbc, dst, src);
    auto tzcnt(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0xf3, 0x0f, 0xbc, dst, src);

    auto lzcnt(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0xf3, 0x0f, 0xbd, dst, src);
    auto lzcnt(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0xf3, 0x0f, 0xbd, dst, src);
    auto lzcnt(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0xf3, 0x0f, 0xbd, dst, src);

    auto mov(T)(T dst, Reg!8 src) if (isRM!(T, 8)) => emit!(0, NA)(0x88, dst, src);
    auto mov(T)(T dst, Reg!16 src) if (isRM!(T, 16)) => emit!(0, NA)(0x89, dst, src);
    auto mov(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NA)(0x89, dst, src);
    auto mov(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NA)(0x89, dst, src);

    auto mov(Reg!8 dst, Address!8 src) => emit!(0, NA)(0x8a, dst, src);
    auto mov(Reg!16 dst, Address!16 src) => emit!(0, NA)(0x8b, dst, src);
    auto mov(Reg!32 dst, Address!32 src) => emit!(0, NA)(0x8b, dst, src);
    auto mov(Reg!64 dst, Address!64 src) => emit!(0, NA)(0x8b, dst, src);
    
    auto mov(Reg!8 dst, ubyte imm8) => emit!(0, NA)(0xb0, encodable(dst), imm8);
    auto mov(Reg!16 dst, ushort imm16) => emit!(0, NA)(0xb8, encodable(dst), imm16);
    auto mov(Reg!32 dst, uint imm32) => emit!(0, NA)(0xb8, encodable(dst), imm32);
    auto mov(Reg!64 dst, ulong imm64) => emit!(0, NA)(0xb8, encodable(dst), imm64);

    auto mov(Address!8 dst, ubyte imm8) => emit!(0, NA)(0xc6, dst, imm8);
    auto mov(Address!16 dst, ushort imm16) => emit!(0, NA)(0xc7, dst, imm16);
    auto mov(Address!32 dst, uint imm32) => emit!(0, NA)(0xc7, dst, imm32);
    auto mov(Address!64 dst, uint imm32) => emit!(0, NA)(0xc7, dst, imm32);

    auto mov(Reg!32 dst, Reg!(-1) src) => emit!(0, NA)(0x0f, 0x20, dst, src);
    auto mov(Reg!64 dst, Reg!(-1) src) => emit!(0, NA)(0x0f, 0x20, dst, src);
    auto mov(Reg!(-1) dst, Reg!32 src) => emit!(0, NA)(0x0f, 0x22, dst, src);
    auto mov(Reg!(-1) dst, Reg!64 src) => emit!(0, NA)(0x0f, 0x22, dst, src);

    auto mov(Reg!32 dst, Reg!(-2) src) => emit!(0, NA)(0x0f, 0x21, dst, src);
    auto mov(Reg!64 dst, Reg!(-2) src) => emit!(0, NA)(0x0f, 0x21, dst, src);
    auto mov(Reg!(-2) dst, Reg!32 src) => emit!(0, NA)(0x0f, 0x23, dst, src);
    auto mov(Reg!(-2) dst, Reg!64 src) => emit!(0, NA)(0x0f, 0x23, dst, src);

    auto movsx(T)(Reg!16 dst, T src) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0xbe, dst, src);
    auto movsx(T)(Reg!32 dst, T src) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0xbe, dst, src);
    auto movsx(T)(Reg!64 dst, T src) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0xbe, dst, src);

    auto movsx(T)(Reg!32 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xbf, dst, src);
    auto movsx(T)(Reg!64 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xbf, dst, src);

    auto movsxd(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x63, dst, src);
    auto movsxd(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x63, dst, src);
    auto movsxd(T)(Reg!64 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x63, dst, src);

    auto movzx(T)(Reg!16 dst, T src) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0xb6, dst, src);
    auto movzx(T)(Reg!32 dst, T src) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0xb6, dst, src);
    auto movzx(T)(Reg!64 dst, T src) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0xb6, dst, src);

    auto movzx(T)(Reg!32 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xb7, dst, src);
    auto movzx(T)(Reg!64 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0xb7, dst, src);

    auto call(ushort rel16) => emit!(0, NA)(0xe8, rel16);
    auto call(uint rel32) => emit!(0, NA)(0xe8, rel32);

    auto call(Reg!16 dst) => emit!(2, NA)(0xff, dst);
    auto call(Reg!32 dst) => emit!(2, NA)(0xff, dst);
    auto call(Reg!64 dst) => emit!(2, NA)(0xff, dst);

    auto call(Address!16 dst) => emit!(3, NA)(0xff, dst);
    auto call(Address!32 dst) => emit!(3, NA)(0xff, dst);
    auto call(Address!64 dst) => emit!(3, NA)(0xff, dst);

    auto loop(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loop");
    auto loope(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loope");
    auto loopne(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loopne");

    auto jmp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jmp");
    auto jmp(T)(T dst) if (isRM!(T, 16)) => emit!(4, NA)(0xff, dst);
    auto jmp(T)(T dst) if (isRM!(T, 32)) => emit!(4, NA)(0xff, dst);
    auto jmp(T)(T dst) if (isRM!(T, 64)) => emit!(4, NA)(0xff, dst);

    auto jmp(Address!16 dst) => emit!(5, NA)(0xff, dst);
    auto jmp(Address!32 dst) => emit!(5, NA)(0xff, dst);
    auto jmp(Address!64 dst) => emit!(5, NA)(0xff, dst);

    auto jmp(ushort imm16) => emit!(0, NA)(0xea, imm16);
    auto jmp(uint imm32) => emit!(0, NA)(0xea, imm32);

    auto ja(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "ja");
    auto jae(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jae");
    auto jb(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jb");
    auto jbe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jbe");
    auto jc(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jc");
    auto jcxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jcxz");
    auto jecxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jecxz");
    auto jrcxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jrcxz");
    auto je(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "je");
    auto jg(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jg");
    auto jge(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jge");
    auto jl(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jl");
    auto jle(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jle");
    auto jna(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jna");
    auto jnae(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnae");
    auto jnb(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnb");
    auto jnbe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnbe");
    auto jnc(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnc");
    auto jne(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jne");
    auto jng(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jng");
    auto jnge(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnge");
    auto jnl(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnl");
    auto jnle(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnle");
    auto jno(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jno");
    auto jnp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnp");
    auto jns(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jns");
    auto jnz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnz");
    auto jo(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jo");
    auto jp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jp");
    auto jpe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jpe");
    auto jpo(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jpo");
    auto js(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "js");
    auto jz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jz");

    auto lfence() => emit!(0, NA)(0x0f, 0xae, 0xe8);
    auto sfence() => emit!(0, NA)(0x0f, 0xae, 0xf8);
    auto mfence() => emit!(0, NA)(0x0f, 0xae, 0xf0);
        
    auto rep(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    auto repe(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    auto repz(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    auto repne(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    auto repnz(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    auto movs(Address!8 dst, Address!8 src) => emit!(0, NA)(0xa4, dst, src);
    auto movs(Address!16 dst, Address!16 src) => emit!(0, NA)(0xa5, dst, src);
    auto movs(Address!32 dst, Address!32 src) => emit!(0, NA)(0xa5, dst, src);
    auto movs(Address!64 dst, Address!64 src) => emit!(0, NA)(0xa5, dst, src);

    auto movsb() => emit!(0, NA)(0xa4);
    auto movsw() => emit!(0, NA)(0x66, 0xa5);
    auto movsd() => emit!(0, NA)(0xa5);
    auto movsq() => emit!(0, NA)(0x48, 0xa5);

    auto cmps(Address!8 dst, Address!8 src) => emit!(0, NA)(0xa6, dst, src);
    auto cmps(Address!16 dst, Address!16 src) => emit!(0, NA)(0xa7, dst, src);
    auto cmps(Address!32 dst, Address!32 src) => emit!(0, NA)(0xa7, dst, src);
    auto cmps(Address!64 dst, Address!64 src) => emit!(0, NA)(0xa7, dst, src);

    auto cmpsb() => emit!(0, NA)(0xa6);
    auto cmpsw() => emit!(0, NA)(0x66, 0xa7);
    auto cmpsd() => emit!(0, NA)(0xa7);
    auto cmpsq() => emit!(0, NA)(0x48, 0xa7);

    auto scas(Address!8 dst) => emit!(0, NA)(0xae, dst);
    auto scas(Address!16 dst) => emit!(0, NA)(0xaf, dst);
    auto scas(Address!32 dst) => emit!(0, NA)(0xaf, dst);
    auto scas(Address!64 dst) => emit!(0, NA)(0xaf, dst);

    auto scasb() => emit!(0, NA)(0xae);
    auto scasw() => emit!(0, NA)(0x66, 0xaf);
    auto scasd() => emit!(0, NA)(0xaf);
    auto scasq() => emit!(0, NA)(0x48, 0xaf);

    auto lods(Address!8 dst) => emit!(0, NA)(0xac, dst);
    auto lods(Address!16 dst) => emit!(0, NA)(0xad, dst);
    auto lods(Address!32 dst) => emit!(0, NA)(0xad, dst);
    auto lods(Address!64 dst) => emit!(0, NA)(0xad, dst);

    auto lodsb() => emit!(0, NA)(0xac);
    auto lodsw() => emit!(0, NA)(0x66, 0xad);
    auto lodsd() => emit!(0, NA)(0xad);
    auto lodsq() => emit!(0, NA)(0x48, 0xad);

    auto stos(Address!8 dst) => emit!(0, NA)(0xaa, dst);
    auto stos(Address!16 dst) => emit!(0, NA)(0xab, dst);
    auto stos(Address!32 dst) => emit!(0, NA)(0xab, dst);
    auto stos(Address!64 dst) => emit!(0, NA)(0xab, dst);

    auto stosb() => emit!(0, NA)(0xaa);
    auto stosw() => emit!(0, NA)(0x66, 0xab);
    auto stosd() => emit!(0, NA)(0xab);
    auto stosq() => emit!(0, NA)(0x48, 0xab);

    auto inal(ubyte imm8) => emit!(0, NA)(0xe4, imm8);
    auto _in(ubyte imm8) => emit!(0, NA)(0xe5, imm8);
    auto inal() => emit!(0, NA)(0xec);
    auto _in() => emit!(0, NA)(0xed);

    auto ins(Address!8 dst) => emit!(0, NA)(0x6c, dst);
    auto ins(Address!16 dst) => emit!(0, NA)(0x6d, dst);
    auto ins(Address!32 dst) => emit!(0, NA)(0x6d, dst);

    auto insb() => emit!(0, NA)(0x6c);
    auto insw() => emit!(0, NA)(0x66, 0x6d);
    auto insd() => emit!(0, NA)(0x6d);
    
    auto outal(ubyte imm8) => emit!(0, NA)(0xe6, imm8);
    auto _out(ubyte imm8) => emit!(0, NA)(0xe7, imm8);
    auto outal() => emit!(0, NA)(0xee);
    auto _out() => emit!(0, NA)(0xef);

    auto outs(Address!8 dst) => emit!(0, NA)(0x6e, dst);
    auto outs(Address!16 dst) => emit!(0, NA)(0x6f, dst);
    auto outs(Address!32 dst) => emit!(0, NA)(0x6f, dst);

    auto outsb() => emit!(0, NA)(0x6e);
    auto outsw() => emit!(0, NA)(0x66, 0x6f);
    auto outsd() => emit!(0, NA)(0x6f);

    auto invvpid(Reg!64 dst, Address!128 src) => emit!(0, NA)(0x66, 0x0f, 0x38, 0x81, dst, src);
    auto invvpid(Reg!32 dst, Address!128 src) => emit!(0, NA)(0x66, 0x0f, 0x38, 0x81, dst, src);
    auto invept(Reg!64 dst, Address!128 src) => emit!(0, NA)(0x66, 0x0f, 0x38, 0x80, dst, src);
    auto invept(Reg!32 dst, Address!128 src) => emit!(0, NA)(0x66, 0x0f, 0x38, 0x80, dst, src);

    auto vmcall() => emit!(0, NA)(0x0f, 0x01, 0xc1);
    auto vmfunc() => emit!(0, NA)(0x0f, 0x01, 0xd4);
    auto vmclear(T)(T dst) if (isRM!(T, 64)) => emit!(6, NA)(0x66, 0x0f, 0xc7, dst);
    auto vmlaunch() => emit!(0, NA)(0x0f, 0x01, 0xc2);
    auto vmresume() => emit!(0, NA)(0x0f, 0x01, 0xc3);
    auto vmxoff() => emit!(0, NA)(0x0f, 0x01, 0xc4);
    auto vmxon(T)(T dst) if (isRM!(T, 64)) => emit!(6, NA)(0xf3, 0x0f, 0xc7, dst);
    
    auto vmwrite(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NP)(0x0f, 0x79, dst, src);
    auto vmwrite(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NP)(0x0f, 0x79, dst, src);
    auto vmread(T)(T dst, Reg!64 src) if (isRM!(T, 64)) => emit!(0, NP)(0x0f, 0x78, dst, src);
    auto vmread(T)(T dst, Reg!32 src) if (isRM!(T, 32)) => emit!(0, NP)(0x0f, 0x78, dst, src);

    auto vmptrst(T)(T dst) if (isRM!(T, 64)) => emit!(7, NP)(0x0f, 0xc7, dst);
    auto vmptrld(T)(T dst) if (isRM!(T, 64)) => emit!(6, NP)(0x0f, 0xc7, dst);

    auto seta(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x97, dst);
    auto setae(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x93, dst);
    auto setb(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x92, dst);
    auto setbe(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x96, dst);
    auto setc(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x92, dst);
    auto sete(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x94, dst);
    auto setg(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9f, dst);
    auto setge(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9d, dst);
    auto setl(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9c, dst);
    auto setle(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9e, dst);
    auto setna(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x96, dst);
    auto setnae(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x92, dst);
    auto setnb(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x93, dst);
    auto setnbe(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x97, dst);
    auto setnc(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x93, dst);
    auto setne(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x95, dst);
    auto setng(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9e, dst);
    auto setnge(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9c, dst);
    auto setnl(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9d, dst);
    auto setnle(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9f, dst);
    auto setno(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x91, dst);
    auto setnp(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9b, dst);
    auto setns(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x99, dst);
    auto setnz(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x95, dst);
    auto seto(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x90, dst);
    auto setp(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9a, dst);
    auto setpe(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9a, dst);
    auto setpo(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x9b, dst);
    auto sets(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x98, dst);
    auto setz(T)(T dst) if (isRM!(T, 8)) => emit!(0, NA)(0x0f, 0x94, dst);
    
    auto cmova(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x47, dst, src);
    auto cmova(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x47, dst, src);
    auto cmova(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x47, dst, src);
    
    auto cmovae(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    auto cmovae(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    auto cmovae(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    
    auto cmovb(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    auto cmovb(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    auto cmovb(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    
    auto cmovbe(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x46, dst, src);
    auto cmovbe(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x46, dst, src);
    auto cmovbe(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x46, dst, src);
    
    auto cmovc(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    auto cmovc(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    auto cmovc(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    
    auto cmove(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x44, dst, src);
    auto cmove(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x44, dst, src);
    auto cmove(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x44, dst, src);
    
    auto cmovg(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4f, dst, src);
    auto cmovg(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4f, dst, src);
    auto cmovg(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4f, dst, src);
    
    auto cmovge(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4d, dst, src);
    auto cmovge(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4d, dst, src);
    auto cmovge(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4d, dst, src);
    
    auto cmovl(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4c, dst, src);
    auto cmovl(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4c, dst, src);
    auto cmovl(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4c, dst, src);
    
    auto cmovle(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4e, dst, src);
    auto cmovle(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4e, dst, src);
    auto cmovle(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4e, dst, src);
    
    auto cmovna(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x46, dst, src);
    auto cmovna(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x46, dst, src);
    auto cmovna(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x46, dst, src);
    
    auto cmovnae(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    auto cmovnae(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    auto cmovnae(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x42, dst, src);
    
    auto cmovnb(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    auto cmovnb(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    auto cmovnb(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    
    auto cmovnbe(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x47, dst, src);
    auto cmovnbe(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x47, dst, src);
    auto cmovnbe(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x47, dst, src);
    
    auto cmovnc(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    auto cmovnc(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    auto cmovnc(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x43, dst, src);
    
    auto cmovne(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x45, dst, src);
    auto cmovne(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x45, dst, src);
    auto cmovne(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x45, dst, src);
    
    auto cmovng(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4e, dst, src);
    auto cmovng(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4e, dst, src);
    auto cmovng(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4e, dst, src);
    
    auto cmovnge(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4c, dst, src);
    auto cmovnge(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4c, dst, src);
    auto cmovnge(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4c, dst, src);
    
    auto cmovnl(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4d, dst, src);
    auto cmovnl(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4d, dst, src);
    auto cmovnl(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4d, dst, src);
    
    auto cmovnle(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4f, dst, src);
    auto cmovnle(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4f, dst, src);
    auto cmovnle(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4f, dst, src);
    
    auto cmovno(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x41, dst, src);
    auto cmovno(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x41, dst, src);
    auto cmovno(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x41, dst, src);
    
    auto cmovnp(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4b, dst, src);
    auto cmovnp(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4b, dst, src);
    auto cmovnp(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4b, dst, src);
    
    auto cmovns(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x49, dst, src);
    auto cmovns(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x49, dst, src);
    auto cmovns(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x49, dst, src);
    
    auto cmovnz(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x45, dst, src);
    auto cmovnz(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x45, dst, src);
    auto cmovnz(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x45, dst, src);
    
    auto cmovo(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x40, dst, src);
    auto cmovo(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x40, dst, src);
    auto cmovo(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x40, dst, src);
    
    auto cmovp(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4a, dst, src);
    auto cmovp(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4a, dst, src);
    auto cmovp(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4a, dst, src);
    
    auto cmovpe(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4a, dst, src);
    auto cmovpe(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4a, dst, src);
    auto cmovpe(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4a, dst, src);
    
    auto cmovpo(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x4b, dst, src);
    auto cmovpo(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x4b, dst, src);
    auto cmovpo(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x4b, dst, src);
    
    auto cmovs(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x48, dst, src);
    auto cmovs(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x48, dst, src);
    auto cmovs(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x48, dst, src);
    
    auto cmovz(T)(Reg!16 dst, T src) if (isRM!(T, 16)) => emit!(0, NA)(0x0f, 0x44, dst, src);
    auto cmovz(T)(Reg!32 dst, T src) if (isRM!(T, 32)) => emit!(0, NA)(0x0f, 0x44, dst, src);
    auto cmovz(T)(Reg!64 dst, T src) if (isRM!(T, 64)) => emit!(0, NA)(0x0f, 0x44, dst, src);

    auto pconfig() => emit!(0, NA)(0x0f, 0x01, 0xc5);
    auto ptwrite(T)(T dst) if (isRM!(T, 32)) => emit!(4, NA)(0xf3, 0x0f, 0xae, dst);
    auto ptwrite(T)(T dst) if (isRM!(T, 64)) => emit!(4, NA)(0xf3, 0x0f, 0xae, dst);

    auto fabs() => emit!(0, NA)(0xd9, 0xe1);
    auto fchs() => emit!(0, NA)(0xd9, 0xe0);

    auto fclex() => emit!(0, NA)(0x9b, 0xdb, 0xe2);
    auto fnclex() => emit!(0, NA)(0xdb, 0xe2);

    auto fadd(Address!32 dst) => emit!(0, NP)(0xd8, dst);
    auto fadd(Address!64 dst) => emit!(0, NP)(0xdc, dst);
    auto fadd(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NA)(0xd8, 0xc0, encodable(src));
        else if (src.index == 0)
            emit!(0, NA)(0xdc, 0xc0, encodable(dst));
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto faddp(Reg!(-3) dst) => emit!(0, NA)(0xde, 0xc0, encodable(dst));
    auto fiadd(Address!32 dst) => emit!(0, NP)(0xda, dst);
    auto fiadd(Address!16 dst) => emit!(0, NP)(0xde, dst);

    auto fbld(Address!80 dst) => emit!(4, NP)(0xdf, dst);
    auto fbstp(Address!80 dst) => emit!(6, NP)(0xdf, dst);

    auto fcom(Address!32 dst) => emit!(2, NP)(0xd8, dst);
    auto fcom(Address!64 dst) => emit!(2, NP)(0xdc, dst);
    auto fcom(Reg!(-3) dst) => emit!(2, NA)(0xd8, 0xd0, encodable(dst));

    auto fcomp(Address!32 dst) => emit!(3, NP)(0xd8, dst);
    auto fcomp(Address!64 dst) => emit!(3, NP)(0xdc, dst);
    auto fcomp(Reg!(-3) dst) => emit!(2, NA)(0xd8, 0xd8, encodable(dst));
    auto fcompp() => emit!(0, NA)(0xde, 0xd9);

    auto fcomi(Reg!(-3) dst) => emit!(0, NA)(0xdb, 0xf0, encodable(dst));
    auto fcomip(Reg!(-3) dst) => emit!(0, NA)(0xdf, 0xf0, encodable(dst));
    auto fucomi(Reg!(-3) dst) => emit!(0, NA)(0xdb, 0xe8, encodable(dst));
    auto fucomip(Reg!(-3) dst) => emit!(0, NA)(0xdf, 0xe8, encodable(dst));

    auto ficom(Address!16 dst) => emit!(2, NP)(0xde, dst);
    auto ficom(Address!32 dst) => emit!(2, NP)(0xda, dst);
    auto ficomp(Address!16 dst) => emit!(2, NP)(0xde, dst);
    auto ficomp(Address!32 dst) => emit!(2, NP)(0xda, dst);
    
    auto fucom(Reg!(-3) dst) => emit!(2, NA)(0xdd, 0xe0, encodable(dst));
    auto fucomp(Reg!(-3) dst) => emit!(2, NA)(0xdd, 0xe8, encodable(dst));
    auto fucompp() => emit!(0, NA)(0xda, 0xe9);

    auto ftst() => emit!(0, NA)(0xd9, 0xe4);

    auto f2xm1() => emit!(0, NA)(0xd9, 0xf0);
    auto fyl2x() => emit!(0, NA)(0xd9, 0xf1);
    auto fyl2xp1() => emit!(0, NA)(0xd9, 0xf9);

    auto fcos() => emit!(0, NA)(0xd9, 0xff);
    auto fsin() => emit!(0, NA)(0xd9, 0xfe);
    auto fsincos() => emit!(0, NA)(0xd9, 0xfb);
    auto fsqrt() => emit!(0, NA)(0xd9, 0xfa);
    
    auto fptan() => emit!(0, NA)(0xd9, 0xf2);
    auto fpatan() => emit!(0, NA)(0xd9, 0xf3);
    auto fprem() => emit!(0, NA)(0xd9, 0xf8);
    auto fprem1() => emit!(0, NA)(0xd9, 0xf5);

    auto fdecstp() => emit!(0, NA)(0xd9, 0xf6);
    auto fincstp() => emit!(0, NA)(0xd9, 0xf7);

    auto fild(Address!16 dst) => emit!(0, NP)(0xdf, dst);
    auto fild(Address!32 dst) => emit!(0, NP)(0xdb, dst);
    auto fild(Address!64 dst) => emit!(5, NP)(0xdf, dst);

    auto fist(Address!16 dst) => emit!(2, NP)(0xdf, dst);
    auto fist(Address!32 dst) => emit!(2, NP)(0xdb, dst);

    auto fistp(Address!16 dst) => emit!(3, NP)(0xdf, dst);
    auto fistp(Address!32 dst) => emit!(3, NP)(0xdb, dst);
    auto fistp(Address!64 dst) => emit!(7, NP)(0xdf, dst);

    auto fisttp(Address!16 dst) => emit!(1, NP)(0xdf, dst);
    auto fisttp(Address!32 dst) => emit!(1, NP)(0xdb, dst);
    auto fisttp(Address!64 dst) => emit!(1, NP)(0xdd, dst);

    auto fldcw(Address!16 dst) => emit!(5, NP)(0xd9, dst);
    auto fstcw(Address!16 dst) => emit!(7, NP)(0x9b, 0xd9, dst);
    auto fnstcw(Address!16 dst) => emit!(7, NP)(0xd9, dst);

    auto fldenv(Address!112 dst) => emit!(4, NP)(0xd9, dst);
    auto fldenv(Address!224 dst) => emit!(4, NP)(0xd9, dst);
    auto fstenv(Address!112 dst) => emit!(6, NP)(0x9b, 0xd9, dst);
    auto fstenv(Address!224 dst) => emit!(6, NP)(0x9b, 0xd9, dst);
    auto fnstenv(Address!112 dst) => emit!(6, NP)(0xd9, dst);
    auto fnstenv(Address!224 dst) => emit!(6, NP)(0xd9, dst);

    auto fstsw(Address!16 dst) => emit!(7, NP)(0x9b, 0xdd, dst);
    auto fstsw() => emit!(0, NA)(0x9b, 0xdf, 0xe0);
    auto fnstsw(Address!16 dst) => emit!(7, NP)(0xdd, dst);
    auto fnstsw() => emit!(0, NA)(0xdf, 0xe0);

    auto fld(Address!32 dst) => emit!(0, NP)(0xd9, dst);
    auto fld(Address!64 dst) => emit!(0, NP)(0xdd, dst);
    auto fld(Address!80 dst) => emit!(5, NP)(0xdb, dst);
    auto fld(Reg!(-3) dst) => emit!(0, NA)(0xd9, 0xc0, encodable(dst));

    auto fld1() => emit!(0, NA)(0xd9, 0xe8);
    auto fldl2t() => emit!(0, NA)(0xd9, 0xe9);
    auto fldl2e() => emit!(0, NA)(0xd9, 0xea);
    auto fldpi() => emit!(0, NA)(0xd9, 0xeb);
    auto fldlg2() => emit!(0, NA)(0xd9, 0xec);
    auto fldln2() => emit!(0, NA)(0xd9, 0xed);
    auto fldz() => emit!(0, NA)(0xd9, 0xee);

    auto fst(Address!32 dst) => emit!(2, NP)(0xd9, dst);
    auto fst(Address!64 dst) => emit!(2, NP)(0xdd, dst);
    auto fst(Reg!(-3) dst) => emit!(0, NA)(0xdd, 0xd0, encodable(dst));
    
    auto fstp(Address!32 dst) => emit!(3, NP)(0xd9, dst);
    auto fstp(Address!64 dst) => emit!(3, NP)(0xdd, dst);
    auto fstp(Address!80 dst) => emit!(7, NP)(0xdb, dst);
    auto fstp(Reg!(-3) dst) => emit!(0, NA)(0xdd, 0xd8, encodable(dst));

    auto fdiv(Address!32 dst) => emit!(6, NP)(0xd8, dst);
    auto fdiv(Address!64 dst) => emit!(6, NP)(0xdc, dst);
    auto fdiv(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NA)(0xd8, 0xf0, encodable(src));
        else if (src.index == 0)
            emit!(0, NA)(0xdc, 0xf8, encodable(dst));
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fdivp(Reg!(-3) dst) => emit!(0, NA)(0xde, 0xf8, encodable(dst));
    auto fidiv(Address!32 dst) => emit!(6, NP)(0xda, dst);
    auto fidiv(Address!16 dst) => emit!(6, NP)(0xde, dst);

    auto fdivr(Address!32 dst) => emit!(7, NP)(0xd8, dst);
    auto fdivr(Address!64 dst) => emit!(7, NP)(0xdc, dst);
    auto fdivr(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NA)(0xd8, 0xf8, encodable(src));
        else if (src.index == 0)
            emit!(0, NA)(0xdc, 0xf0, encodable(dst));
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fdivrp(Reg!(-3) dst) => emit!(0, NA)(0xde, 0xf0, encodable(dst));
    auto fidivr(Address!32 dst) => emit!(7, NP)(0xda, dst);
    auto fidivr(Address!16 dst) => emit!(7, NP)(0xde, dst);

    auto fscale() => emit!(0, NA)(0xd9, 0xfd);
    auto frndint() => emit!(0, NA)(0xd9, 0xfc);
    auto fexam() => emit!(0, NA)(0xd9, 0xe5);
    auto ffree(Reg!(-3) dst) => emit!(0, NA)(0xdd, 0xc0, encodable(dst));
    auto fxch(Reg!(-3) dst) => emit!(0, NA)(0xd9, 0xc8, encodable(dst));
    auto fxtract() => emit!(0, NA)(0xd9, 0xf4);

    auto fnop() => emit!(0, NA)(0xd9, 0xd0);
    auto fninit() => emit!(0, NA)(0x9b, 0xdb, 0xe3);
    auto finit() => emit!(0, NA)(0xdb, 0xe3);

    auto fsave(Address!752 dst) => emit!(6, NA)(0x9b, 0xdd, dst);
    auto fsave(Address!864 dst) => emit!(6, NA)(0x9b, 0xdd, dst);
    auto fnsave(Address!752 dst) => emit!(6, NA)(0xdd, dst);
    auto fnsave(Address!864 dst) => emit!(6, NA)(0xdd, dst);

    auto frstor(Address!752 dst) => emit!(4, NA)(0xdd, dst);
    auto frstor(Address!864 dst) => emit!(4, NA)(0xdd, dst);

    static if (!X64)
    auto fxsave(Address!4096 dst) => emit!(0, NP)(0x0f, 0xae, dst);
    static if (X64)
    auto fxsave(Address!4096 dst) => emit!(0, NP)(0x48, 0x0f, 0xae, dst);
    
    static if (!X64)
    auto fxrstor(Address!4096 dst) => emit!(1, NP)(0x0f, 0xae, dst);
    static if (X64)
    auto fxrstor(Address!4096 dst) => emit!(1, NP)(0x48, 0x0f, 0xae, dst);

    auto fmul(Address!32 dst) => emit!(1, NP)(0xd8, dst);
    auto fmul(Address!64 dst) => emit!(1, NP)(0xdc, dst);
    auto fmul(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NA)(0xd8, 0xc8, encodable(src));
        else if (src.index == 0)
            emit!(0, NA)(0xdc, 0xc8, encodable(dst));
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fmulp(Reg!(-3) dst) => emit!(0, NA)(0xde, 0xc8, encodable(dst));
    auto fimul(Address!32 dst) => emit!(1, NP)(0xda, dst);
    auto fimul(Address!16 dst) => emit!(1, NP)(0xde, dst);

    auto fsub(Address!32 dst) => emit!(4, NP)(0xd8, dst);
    auto fsub(Address!64 dst) => emit!(4, NP)(0xdc, dst);
    auto fsub(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NA)(0xd8, 0xe0, encodable(src));
        else if (src.index == 0)
            emit!(0, NA)(0xdc, 0xe8, encodable(dst));
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fsubp(Reg!(-3) dst) => emit!(0, NA)(0xde, 0xe8, encodable(dst));
    auto fisub(Address!32 dst) => emit!(4, NP)(0xda, dst);
    auto fisub(Address!16 dst) => emit!(4, NP)(0xde, dst);

    auto fsubr(Address!32 dst) => emit!(5, NP)(0xd8, dst);
    auto fsubr(Address!64 dst) => emit!(5, NP)(0xdc, dst);
    auto fsubr(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NA)(0xd8, 0xe8, encodable(src));
        else if (src.index == 0)
            emit!(0, NA)(0xdc, 0xe0, encodable(dst));
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fsubrp(Reg!(-3) dst) => emit!(0, NA)(0xde, 0xe0, encodable(dst));
    auto fisubr(Address!32 dst) => emit!(5, NP)(0xda, dst);
    auto fisubr(Address!16 dst) => emit!(5, NP)(0xde, dst);

    auto fcmovb(Reg!(-3) dst) => emit!(0, NA)(0xda, 0xc0, encodable(dst));
    auto fcmove(Reg!(-3) dst) => emit!(0, NA)(0xda, 0xc8, encodable(dst));
    auto fcmovbe(Reg!(-3) dst) => emit!(0, NA)(0xda, 0xd0, encodable(dst));
    auto fcmovu(Reg!(-3) dst) => emit!(0, NA)(0xda, 0xd8, encodable(dst));
    auto fcmovnb(Reg!(-3) dst) => emit!(0, NA)(0xdb, 0xc0, encodable(dst));
    auto fcmovne(Reg!(-3) dst) => emit!(0, NA)(0xdb, 0xc8, encodable(dst));
    auto fcmovnbe(Reg!(-3) dst) => emit!(0, NA)(0xdb, 0xd0, encodable(dst));
    auto fcmovnu(Reg!(-3) dst) => emit!(0, NA)(0xdb, 0xd8, encodable(dst));

    Address!8 bytePtr(Args...)(Args args)
    {
        return Address!8(args);
    }

    Address!16 wordPtr(Args...)(Args args)
    {
        return Address!16(args);
    }

    Address!32 dwordPtr(Args...)(Args args)
    {
        return Address!32(args);
    }

    Address!64 qwordPtr(Args...)(Args args)
    {
        return Address!64(args);
    }

    Address!128 xmmwordPtr(Args...)(Args args)
    {
        return Address!128(args);
    }

    Address!256 ymmwordPtr(Args...)(Args args)
    {
        return Address!256(args);
    }

    Address!512 zmmwordPtr(Args...)(Args args)
    {
        return Address!512(args);
    }
}