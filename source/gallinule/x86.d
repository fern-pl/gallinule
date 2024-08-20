// Originally based on: https://github.com/philpax/djitt
module gallinule.x86;

import std.bitmanip;
import std.traits;
import tern.traits : Attributes;
import std.typecons;
import tern.algorithm;
import tern.state;

/* ====== ADDRESSING ====== */

private enum Mode
{
    Memory,
    MemoryOffset8,
    MemoryOffsetExt,
    Register
}

private union ModRM
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

private:
ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst)
    if (isInstanceOf!(Mem, SRC) && isInstanceOf!(Reg, DST))
{
    if (src.size == 0)
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

ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst, Mode mod = Mode.Register)
    if (isInstanceOf!(Reg, SRC) && isInstanceOf!(Reg, DST))
{
    ModRM generateModRM;
    generateModRM.src = (src.index % 8);
    generateModRM.dst = (dst.index % 8) | OP;
    generateModRM.mod = cast(ubyte)mod;
    return [generateModRM];
}

ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst)
    if (isInstanceOf!(Mem, SRC) && isInstanceOf!(Mem, DST))
{
    return generateModRM!OP(Reg!(TemplateArgsOf!(DST))(dst.register), Reg!(TemplateArgsOf!(SRC))(src.register));
}

ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst)
    if (isInstanceOf!(Reg, SRC) && isInstanceOf!(Mem, DST))
{
    return generateModRM!OP(dst, src);
}

/// This is simply used for constraining T to be an address or register of the given size(s).
enum valid(T, short SIZE) = is(T == Reg!SIZE) || is(T == Mem!SIZE);
enum valid(T, short RS, short AS) = is(T == Reg!RS) || is(T == Mem!AS);

enum M = 0;
// Used for generating instructions with directly encoded registers.
enum NRM = 1;
// Used for generating instructions without REX prefixes.
enum NP = 2;
enum VEX = 3;
// Used for generating integer VEX instructions.
enum VEXI = 4;
enum EVEX = 5;
enum MVEX = 6;
// Exactly the same as NP except flips dst and src.
enum SSE = 7;

// map_select
enum XOP = 0;
enum DEFAULT = 1;
enum F38 = 2;
enum F3A = 3;
enum MSR = 7;

/**
    alias CR = Reg!(-1);
    alias DR = Reg!(-2);
    alias ST = Reg!(-3);
    alias R8 = Reg!8;
    alias R16 = Reg!16;
    alias R32 = Reg!32;
    alias R64 = Reg!64;
    alias MMX = Reg!64;
    alias XMM = Reg!128;
    alias YMM = Reg!256;
    alias ZMM = Reg!512;

    Addresses: Mem!(SIZE)

    If the instruction is an integer instruction: use VEXI, otherwise use VEX (like emit!(MODRM_OR, KIND)),
    Emits are in the format emit(ubyte OP, ubyte SELECTOR = M, ubyte SIZE = 128, ubyte MAP = DEFAULT, ubyte PREFIX = 0)
    Map selection is specified by the part of the VEX prefix in docs after the width, ie:
        VEX.256.0F
            SIZE = 256
            MAP = 0F = DEFAULT
            PREFIX = 0
        VEX.128.0F3A.66
            SIZE = 128
            MAP = F3A
            PREFIX = 66
*/

/* ====== FRONT-END ====== */

public struct Reg(short SIZE)
{
public:
final:
    ubyte index;
    bool extended;
}

public struct Mem(short SIZE)
{
public:
final:
    short size;
    ubyte register;
    uint offset;
    ubyte segment = ds;

    this(T)(T register, ubyte segment, uint offset = 0)
        if (isInstanceOf!(Reg, T))
    {
        this.size = TemplateArgsOf!(T)[0];
        this.register = register.index;
        this.offset = offset;
        this.segment = segment;
    }

    this(T)(T register, uint offset = 0)
        if (isInstanceOf!(Reg, T))
    {
        this.size = TemplateArgsOf!(T)[0];
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

public enum CRID
{
    VME,
    PVI,
    TSD,
    DE,
    PSE,
    PAE,
    MCE,
    PGE,
    PCE,
    OSFXSR,
    OSXMMEXCPT,
    UMIP,
    // RESERVED
    VMXE = 13,
    SMXE,
    // RESERVED
    FSGSBASE = 16,
    PCIDE,
    OSXSAVE,
    // RESERVED
    SMEP = 20,
    SMAP,
    PKE,
    CET,
    PKS,
    UINTR
}

public enum CPUID7_EBX
{
    FSGSBASE,
    TSC_ADJUST,
    SGX,
    // LZCNT and TZCNT
    BMI1, 
    // XACQUIRE, XRELEASE, XTEST
    HLE,
    AVX2,
    FPDP,
    SMEP,
    BMI2,
    ERMS,
    // INVPCID
    INVPCID, 
    // XBEGIN, XABORT, XEND and XTEST
    RTM, 
    PQM,
    FPCSDS,
    // BND*/BOUND
    MPX, 
    PQE,
    AVX512F,
    AVX512DQ,
    // RDSEED
    RDSEED,
    // ADCX and ADOX
    ADX, 
    // CLAC and STAC
    SMAP, 
    AVX512IFMA,
    PCOMMIT, 
    // CLFLUSHOPT
    CLFLUSHOPT, 
    // CLWB
    CLWB,
    // PTWRITE 
    PT, 
    AVX512PF,
    AVX512ER,
    AVX512CD,
    SHA,
    AVX512BW,
    AVX512VL
}

public enum CPUID7_ECX
{
    PREFETCHWT1,
    AVX512VBMI,
    UMIP,
    PKU,
    OSPKE,
    AVX512VBMI2 = 6,
    // INCSSP, RDSSP, SAVESSP, RSTORSSP, SETSSBSY, CLRSSBSY, WRSS, WRUSS, ENDBR64, and ENDBR64
    CET,
    GFNI,
    VAES,
    VPCL,
    AVX512VNNI,
    AVX512BITALG,
    TME,
    // VPOPCNT{D,Q}
    AVX512VP,
    VA57 = 16,
    RDPID = 22,
    SGX_LC = 30
}

public enum CPUID7_EDX
{
    AVX512QVNNIW = 2,
    AVX512QFMA = 3,
    PCONFIG = 18,
    IBRS_IBPB = 26,
    STIBP = 27
}

public enum CPUID1_ECX
{
    // FISTTP
    SSE3,
    // PCLMULQDQ
    PCLMUL,
    DTES64,
    // MONITOR/MWAIT
    MON,
    DSCPL,
    // VM*
    VMX,
    SMX,
    EST,
    TM2,
    SSSE3,
    CID,
    SDBG,
    FMA,
    // CMPXCHG16B
    CX16,
    XTPR,
    PDCM,
    PCID,
    DCA,
    SSE4_1,
    SSE4_2,
    X2APIC,
    // MOVBE
    MOVBE,
    // POPCNT
    POPCNT,
    TSCD,
    // AES*
    AES,
    // XGETBV, XSETBV, XSAVEOPT, XSAVE, and XRSTOR
    XSAVE,
    OSXSAVE,
    AVX,
    // VCVTPH2PS and VCVTPS2PH
    F16C,
    // RDRAND
    RDRAND,
    HV
}

public enum CPUID1_EDX
{
    FPU,
    VME,
    DE,
    PSE,
    // RDTSC
    TSC,
    // RDMSR/WRMSR
    MSR,
    PAE,
    // CMPXCHG8B
    CX8,
    APIC,
    // SYSENTER/SYSEXIT
    SEP,
    MTRR,
    PGE,
    MCA,
    // CMOVcc
    CMOV,
    PAT,
    PSE36,
    PSN,
    // CLFLUSH
    CLFL,
    DS,
    ACPI,
    MMX,
    // FXSAVE/FXRSTOR
    FXSR,
    NP,
    SSE2,
    SS,
    HTT,
    TM,
    IA64,
    PBE
}

public:
alias CR = Reg!(-1);
alias DR = Reg!(-2);
alias ST = Reg!(-3);
alias R8 = Reg!8;
alias R16 = Reg!16;
alias R32 = Reg!32;
alias R64 = Reg!64;
alias MMX = Reg!64;
alias XMM = Reg!128;
alias YMM = Reg!256;
alias ZMM = Reg!512;

enum cr0 = CR(0);
enum cr2 = CR(2);
enum cr3 = CR(3);
enum cr4 = CR(4);

enum dr0 = DR(0);
enum dr1 = DR(1);
enum dr2 = DR(2);
enum dr3 = DR(3);
enum dr6 = DR(6);
enum dr7 = DR(7);

// ST registers aren't real registers, the FPU uses a stack
enum st0 = ST(0);
enum st1 = ST(1);
enum st2 = ST(2);
enum st3 = ST(3);
enum st4 = ST(4);
enum st5 = ST(5);
enum st6 = ST(6);
enum st7 = ST(7);

enum al = Reg!8(0);
enum cl = Reg!8(1);
enum dl = Reg!8(2);
enum bl = Reg!8(3);
enum ah = Reg!8(4);
enum ch = Reg!8(5);
enum dh = Reg!8(6);
enum bh = Reg!8(7);
enum spl = Reg!8(4, true);
enum bpl = Reg!8(5, true);
enum sil = Reg!8(6, true);
enum dil = Reg!8(7, true);
enum r8b = Reg!8(8);
enum r9b = Reg!8(9);
enum r10b = Reg!8(10);
enum r11b = Reg!8(11);
enum r12b = Reg!8(12);
enum r13b = Reg!8(13);
enum r14b = Reg!8(14);
enum r15b = Reg!8(15);

enum ax = Reg!16(0);
enum cx = Reg!16(1);
enum dx = Reg!16(2);
enum bx = Reg!16(3);
enum sp = Reg!16(4);
enum bp = Reg!16(5);
enum si = Reg!16(6);
enum di = Reg!16(7);
enum r8w = Reg!16(8);
enum r9w = Reg!16(9);
enum r10w = Reg!16(10);
enum r11w = Reg!16(11);
enum r12w = Reg!16(12);
enum r13w = Reg!16(13);
enum r14w = Reg!16(14);
enum r15w = Reg!16(15);

enum eax = Reg!32(0);
enum ecx = Reg!32(1);
enum edx = Reg!32(2);
enum ebx = Reg!32(3);
enum esp = Reg!32(4);
enum ebp = Reg!32(5);
enum esi = Reg!32(6);
enum edi = Reg!32(7);
enum r8d = Reg!32(8);
enum r9d = Reg!32(9);
enum r10d = Reg!32(10);
enum r11d = Reg!32(11);
enum r12d = Reg!32(12);
enum r13d = Reg!32(13);
enum r14d = Reg!32(14);
enum r15d = Reg!32(15);

enum rax = Reg!64(0);
enum rcx = Reg!64(1);
enum rdx = Reg!64(2);
enum rbx = Reg!64(3);
enum rsp = Reg!64(4);
enum rbp = Reg!64(5);
enum rsi = Reg!64(6);
enum rdi = Reg!64(7);
enum r8 = Reg!64(8);
enum r9 = Reg!64(9);
enum r10 = Reg!64(10);
enum r11 = Reg!64(11);
enum r12 = Reg!64(12);
enum r13 = Reg!64(13);
enum r14 = Reg!64(14);
enum r15 = Reg!64(15);

// TODO: This lets you do evil by using Reg!64 MMX as Reg!64 R64
enum mm0 = Reg!64(0);
enum mm1 = Reg!64(1);
enum mm2 = Reg!64(2);
enum mm3 = Reg!64(3);
enum mm4 = Reg!64(4);
enum mm5 = Reg!64(5);
enum mm6 = Reg!64(6);
enum mm7 = Reg!64(7);

enum xmm0 = Reg!128(0);
enum xmm1 = Reg!128(1);
enum xmm2 = Reg!128(2);
enum xmm3 = Reg!128(3);
enum xmm4 = Reg!128(4);
enum xmm5 = Reg!128(5);
enum xmm6 = Reg!128(6);
enum xmm7 = Reg!128(7);
enum xmm8 = Reg!128(8);
enum xmm9 = Reg!128(9);
enum xmm10 = Reg!128(10);
enum xmm11 = Reg!128(11);
enum xmm12 = Reg!128(12);
enum xmm13 = Reg!128(13);
enum xmm14 = Reg!128(14);
enum xmm15 = Reg!128(15);

enum ymm0 = Reg!256(0);
enum ymm1 = Reg!256(1);
enum ymm2 = Reg!256(2);
enum ymm3 = Reg!256(3);
enum ymm4 = Reg!256(4);
enum ymm5 = Reg!256(5);
enum ymm6 = Reg!256(6);
enum ymm7 = Reg!256(7);
enum ymm8 = Reg!256(8);
enum ymm9 = Reg!256(9);
enum ymm10 = Reg!256(10);
enum ymm11 = Reg!256(11);
enum ymm12 = Reg!256(12);
enum ymm13 = Reg!256(13);
enum ymm14 = Reg!256(14);
enum ymm15 = Reg!256(15);

enum zmm0 = Reg!512(0);
enum zmm1 = Reg!512(1);
enum zmm2 = Reg!512(2);
enum zmm3 = Reg!512(3);
enum zmm4 = Reg!512(4);
enum zmm5 = Reg!512(5);
enum zmm6 = Reg!512(6);
enum zmm7 = Reg!512(7);
enum zmm8 = Reg!512(8);
enum zmm9 = Reg!512(9);
enum zmm10 = Reg!512(10);
enum zmm11 = Reg!512(11);
enum zmm12 = Reg!512(12);
enum zmm13 = Reg!512(13);
enum zmm14 = Reg!512(14);
enum zmm15 = Reg!512(15);

enum ubyte es = 0x26;
enum ubyte cs = 0x2e;
enum ubyte ss = 0x36;
enum ubyte ds = 0x3e;
enum ubyte fs = 0x64;
enum ubyte gs = 0x65;

// TODO: SIB
// TODO: Fix parameter names to make them more clear
public struct Block(bool X64)
{
package:
final:
    ptrdiff_t[string] labels;
    Tuple!(ptrdiff_t, string, string, bool)[] branches;
    ubyte[] buffer;

public:
    template emit(ubyte OP, ubyte SELECTOR = M, ubyte SIZE = 128, ubyte MAP = DEFAULT, ubyte PREFIX = 0)
    {
        size_t emit(ARGS...)(ARGS args)
        {
            ubyte[] buffer;
            bool prefixed;
            ptrdiff_t ct = 0;

            bool isRM1(size_t INDEX)()
            {
                static if (INDEX >= ARGS.length)
                    return false;
                else static if (INDEX + 1 >= ARGS.length)
                    return isInstanceOf!(Reg, ARGS[INDEX]) || isInstanceOf!(Mem, ARGS[INDEX]);
                else
                {
                    return (isInstanceOf!(Reg, ARGS[INDEX]) || isInstanceOf!(Mem, ARGS[INDEX])) &&
                        !isInstanceOf!(Reg, ARGS[INDEX + 1]) && !isInstanceOf!(Mem, ARGS[INDEX + 1]);
                }
            }
            
            bool isRM2(size_t INDEX)()
            {
                static if (INDEX + 1 >= ARGS.length)
                    return false;
                else
                    return (isInstanceOf!(Reg, ARGS[INDEX]) || isInstanceOf!(Mem, ARGS[INDEX])) && isRM1!(INDEX + 1);
            }
            
            bool isRM3(size_t INDEX)()
            {
                static if (INDEX + 2 >= ARGS.length)
                    return false;
                else
                    return (isInstanceOf!(Reg, ARGS[INDEX]) || isInstanceOf!(Mem, ARGS[INDEX])) &&
                        (isInstanceOf!(Reg, ARGS[INDEX + 1]) || isInstanceOf!(Mem, ARGS[INDEX + 1])) && isRM1!(INDEX + 2);
            }

            static if (SELECTOR == M || SELECTOR == NRM || SELECTOR == NP || SELECTOR == SSE)
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
                else static if (isInstanceOf!(Mem, SRC))
                {
                    hasRex |= src.register >= 8;
                    w = is(SRC == Mem!64);
                    b = src.register >= 8;
                }
                
                static if (isInstanceOf!(Reg, DST))
                {
                    hasRex |= is(DST == Reg!64) || (is(DST == Reg!8) && dst.extended) || dst.index >= 8;
                    w = is(DST == Reg!64);
                    b = dst.index >= 8;
                }
                else static if (isInstanceOf!(Mem, DST))
                {
                    hasRex |= dst.register >= 8;
                    w = is(DST == Mem!64);
                    x = dst.register >= 8;
                }

                static if (isInstanceOf!(Mem, SRC))
                {
                    if (src.segment != ds)
                        buffer = src.segment~buffer;
                }
                else static if (isInstanceOf!(Mem, DST))
                {
                    if (dst.segment != ds)
                        buffer = dst.segment~buffer;
                }

                static if (SELECTOR != NP && SELECTOR != SSE)
                if (hasRex)
                {
                    ubyte rex = 0b01000000;
                    if (w) rex |= (1 << 3);
                    if (r) rex |= (1 << 2);
                    if (x) rex |= (1 << 1);
                    if (b) rex |= (1 << 0);
                    
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
                
                static if (isInstanceOf!(Mem, SRC))
                {
                    if ((X64 && src.size != 64) || src.size != 32)
                        buffer = 0x67~buffer;
                }

                static if (isInstanceOf!(Mem, DST))
                {
                    if ((X64 && dst.size != 64) || dst.size != 32)
                        buffer = 0x67~buffer;
                }
            }

            static if (SELECTOR == VEX || SELECTOR == VEXI)
            void generatePrefix(SRC, DST, STOR = int)(SRC src, DST dst, STOR stor = STOR.init)
            {
                prefixed = true;
                bool r;
                bool x;
                bool b;
                immutable ubyte map_select = MAP;
                bool we = SELECTOR == VEX;
                ubyte vvvv = 0b1111;
                immutable bool l = SIZE != 128;
                immutable ubyte pp = (PREFIX == 0x66) ? 1 : ((PREFIX == 0xf3) ? 2 : ((PREFIX == 0xf2) ? 3 : 0));

                static if (isInstanceOf!(Reg, STOR))
                {
                    static if (isInstanceOf!(Reg, DST))
                        vvvv = cast(ubyte)~dst.index;
                    else static if (isInstanceOf!(Mem, DST))
                        vvvv = cast(ubyte)~dst.register;

                    dst = DST(stor.index);
                }
                else static if (isInstanceOf!(Mem, STOR))
                {
                    static if (isInstanceOf!(Reg, DST))
                        vvvv = cast(ubyte)~dst.index;
                    else static if (isInstanceOf!(Mem, DST))
                        vvvv = cast(ubyte)~dst.register;
                        
                    dst = DST(stor.register);
                }

                static if (isInstanceOf!(Reg, SRC))
                {
                    static if (SELECTOR == VEXI)
                        we = is(SRC == Reg!64);
                    b = src.index >= 8;
                }
                else static if (isInstanceOf!(Mem, SRC))
                {
                    static if (SELECTOR == VEXI)
                        we = is(SRC == Mem!64);
                    b = src.register >= 8;
                }
                
                static if (isInstanceOf!(Reg, DST))
                {
                    static if (SELECTOR == VEXI)
                        we = is(DST == Reg!64);
                    r = dst.index >= 8;
                }
                else static if (isInstanceOf!(Mem, DST))
                {
                    static if (SELECTOR == VEXI)
                        we = is(DST == Mem!64);
                    x = dst.register >= 8;
                }

                static if (isInstanceOf!(Mem, SRC))
                {
                    if (src.segment != ds)
                        buffer = src.segment~buffer;
                }
                else static if (isInstanceOf!(Mem, DST))
                {
                    if (dst.segment != ds)
                        buffer = dst.segment~buffer;
                }

                ubyte[] vex;
                if (map_select != 1 || r || x || b || MAP == XOP)
                {
                    static if (SELECTOR != VEXI)
                        we = false;

                    vex ~= MAP == XOP ? 0x8f : 0xc4;
                    vex ~= (cast(ubyte)(((r ? 0 : 1) << 5) | ((x ? 0 : 1) << 6) | ((b ? 0 : 1) << 7))) | (map_select & 0b00011111);
                }
                else
                    vex ~= 0xc5;
                vex ~= we << 7 | (vvvv & 0b00001111) << 3 | (l ? 1 : 0) << 2 | (pp & 0b00000011);
                
                buffer = vex~buffer;
                
                static if (isInstanceOf!(Mem, SRC))
                {
                    if ((X64 && src.size != 64) || src.size != 32)
                        buffer = 0x67~buffer;
                }

                static if (isInstanceOf!(Mem, DST))
                {
                    if ((X64 && dst.size != 64) || dst.size != 32)
                        buffer = 0x67~buffer;
                }
            }

            foreach (i, arg; args)
            {
                if (ct-- > 0)
                    continue;

                static if (is(typeof(arg) == int))
                    buffer ~= cast(ubyte)arg;
                else static if (isScalarType!(typeof(arg)))
                    buffer ~= (cast(ubyte*)&arg)[0..typeof(arg).sizeof];
                else static if (is(typeof(arg) == ubyte[]))
                    buffer ~= arg;
                else static if (SELECTOR == NRM && isInstanceOf!(Reg, typeof(arg)))
                {
                    buffer[$-1] += arg.index % 8;
                    generatePrefix(typeof(arg)(0), arg);
                }
                else static if (isRM1!i)
                {
                    auto dst = arg;
                    auto src = Reg!(TemplateArgsOf!(typeof(arg)))(0);
                    static if (SELECTOR == M || SELECTOR == NP || SELECTOR == NRM)
                        buffer ~= generateModRM!OP(dst, src);
                    else
                        buffer ~= generateModRM!OP(src, dst);
                    generatePrefix(src, dst);
                }
                else static if (isRM2!i)
                {
                    auto dst = arg;
                    auto src = args[i + 1];
                    static if (SELECTOR == M || SELECTOR == NP || SELECTOR == NRM)
                        buffer ~= generateModRM!OP(dst, src);
                    else
                        buffer ~= generateModRM!OP(src, dst);
                    generatePrefix(src, dst);
                    ct = 1;
                }
                else static if (isRM3!i)
                {
                    auto dst = args[i + 2];
                    auto src = arg;
                    buffer ~= generateModRM!OP(dst, src);
                    generatePrefix(src, args[i + 1], dst);
                    ct = 2;
                }
                else
                    static assert(0, "May not emit a non-scalar, non-ubyte[] value of type '"~typeof(arg).stringof~"'!");
            }

            if (!prefixed)
            {
                static if (SELECTOR != M && SELECTOR != NP && SELECTOR != NP && SELECTOR != NRM)
                    generatePrefix(Reg!(typeof(args[0]).sizeof * 128)(0), Reg!(typeof(args[0]).sizeof * 128)(0));

                static if (SELECTOR == M || SELECTOR == NP || SELECTOR == NP || SELECTOR == NRM)
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
    }

    ubyte[] finalize()
    {
        immutable static ubyte[][string] branchMap = [
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

        size_t abs;
        size_t calculateBranch(T)(T branch)
        {
            size_t size;
            auto rel = labels[branch[1]] - branch[0] + abs;
            bool isRel8 = rel <= ubyte.max && rel >= ubyte.min;
            bool isRel16 = rel <= ushort.max && rel >= ushort.min;

            if (isRel8)
                size = branchMap[branch[2]~'1'].length + 1;
            else if (isRel16)
                size = branchMap[branch[2]~'2'].length + 2;
            else
                size = branchMap[branch[2]~'4'].length + 4;

            return size;
        }

        foreach (ref i, branch; branches)
        {
            if (i + 1 < branches.length && branches[i + 1][3] && branches[i + 1][0] == branch[0])
                labels[branch[1]] += calculateBranch(branches[i + 1]);

            ubyte[] buffer;

            branch[0] += abs;
            auto rel = labels[branch[1]] - branch[0];
            bool isRel8 = rel <= byte.max && rel >= byte.min;
            bool isRel16 = rel <= short.max && rel >= short.min;

            buffer ~= branchMap[branch[2]~(isRel8 ? '1' : isRel16 ? '2' : '4')];

            if (isRel8)
                buffer ~= cast(ubyte)rel;
            else if (isRel16)
                buffer ~= (cast(ubyte*)&rel)[0..2];
            else
                buffer ~= (cast(ubyte*)&rel)[0..4];

            abs += buffer.length;
            this.buffer = this.buffer[0..branch[0]]~buffer~this.buffer[branch[0]..$];
        }
        branches = null;
        return this.buffer;
    }

    auto label(string name) => labels[name] = buffer.length;
    
    // These categories are intended to separate instructions based on their corresponding flag,
    // however, they do not accurately reflect this and are more whimsical than logical.

    /* ====== PSEUDO/CUSTOM ====== */

    auto cridvme() => mov(rax, cr4) + shr(rax, CRID.VME) + and(rax, 1);
    auto cridpvi() => mov(rax, cr4) + shr(rax, CRID.PVI) + and(rax, 1);
    auto cridtsd() => mov(rax, cr4) + shr(rax, CRID.TSD) + and(rax, 1);
    auto cridde() => mov(rax, cr4) + shr(rax, CRID.DE) + and(rax, 1);
    auto cridpse() => mov(rax, cr4) + shr(rax, CRID.PSE) + and(rax, 1);
    auto cridpae() => mov(rax, cr4) + shr(rax, CRID.PAE) + and(rax, 1);
    auto cridmce() => mov(rax, cr4) + shr(rax, CRID.MCE) + and(rax, 1);
    auto cridpge() => mov(rax, cr4) + shr(rax, CRID.PGE) + and(rax, 1);
    auto cridpce() => mov(rax, cr4) + shr(rax, CRID.PCE) + and(rax, 1);
    auto cridosfxsr() => mov(rax, cr4) + shr(rax, CRID.OSFXSR) + and(rax, 1);
    auto cridosxmmexcpt() => mov(rax, cr4) + shr(rax, CRID.OSXMMEXCPT) + and(rax, 1);
    auto cridumip() => mov(rax, cr4) + shr(rax, CRID.UMIP) + and(rax, 1);
    auto cridvmxe() => mov(rax, cr4) + shr(rax, CRID.VMXE) + and(rax, 1);
    auto cridsmxe() => mov(rax, cr4) + shr(rax, CRID.SMXE) + and(rax, 1);
    auto cridfsgsbase() => mov(rax, cr4) + shr(rax, CRID.FSGSBASE) + and(rax, 1);
    auto cridpcide() => mov(rax, cr4) + shr(rax, CRID.PCIDE) + and(rax, 1);
    auto cridosxsave() => mov(rax, cr4) + shr(rax, CRID.OSXSAVE) + and(rax, 1);
    auto cridsmep() => mov(rax, cr4) + shr(rax, CRID.SMEP) + and(rax, 1);
    auto cridsmap() => mov(rax, cr4) + shr(rax, CRID.SMAP) + and(rax, 1);
    auto cridpke() => mov(rax, cr4) + shr(rax, CRID.PKE) + and(rax, 1);
    auto cridcet() => mov(rax, cr4) + shr(rax, CRID.CET) + and(rax, 1);
    auto cridpks() => mov(rax, cr4) + shr(rax, CRID.PKS) + and(rax, 1);
    auto criduintr() => mov(rax, cr4) + shr(rax, CRID.UINTR) + and(rax, 1);

    auto idavx512vl() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX512VL) + and(ebx, 1);
    auto idavx512bw() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX512BW) + and(ebx, 1);
    auto idsha() => cpuid(7) + shr(ebx, CPUID7_EBX.SHA) + and(ebx, 1);
    auto idavx512cd() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX512CD) + and(ebx, 1);
    auto idavx512er() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX512ER) + and(ebx, 1);
    auto idavx512pf() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX512PF) + and(ebx, 1);
    auto idpt() => cpuid(7) + shr(ebx, CPUID7_EBX.PT) + and(ebx, 1);
    auto idclwb() => cpuid(7) + shr(ebx, CPUID7_EBX.CLWB) + and(ebx, 1);
    auto idclflushopt() => cpuid(7) + shr(ebx, CPUID7_EBX.CLFLUSHOPT) + and(ebx, 1);
    auto idpcommit() => cpuid(7) + shr(ebx, CPUID7_EBX.PCOMMIT) + and(ebx, 1);
    auto idavx512ifma() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX512IFMA) + and(ebx, 1);
    auto idsmap() => cpuid(7) + shr(ebx, CPUID7_EBX.SMAP) + and(ebx, 1);
    auto idadx() => cpuid(7) + shr(ebx, CPUID7_EBX.ADX) + and(ebx, 1);
    auto idrdseed() => cpuid(7) + shr(ebx, CPUID7_EBX.RDSEED) + and(ebx, 1);
    auto idavx512dq() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX512DQ) + and(ebx, 1);
    auto idavx512f() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX512F) + and(ebx, 1);
    auto idpqe() => cpuid(7) + shr(ebx, CPUID7_EBX.PQE) + and(ebx, 1);
    auto idrtm() => cpuid(7) + shr(ebx, CPUID7_EBX.RTM) + and(ebx, 1);
    auto idinvpcid() => cpuid(7) + shr(ebx, CPUID7_EBX.INVPCID) + and(ebx, 1);
    auto iderms() => cpuid(7) + shr(ebx, CPUID7_EBX.ERMS) + and(ebx, 1);
    auto idbmi2() => cpuid(7) + shr(ebx, CPUID7_EBX.BMI2) + and(ebx, 1);
    auto idsmep() => cpuid(7) + shr(ebx, CPUID7_EBX.SMEP) + and(ebx, 1);
    auto idfpdp() => cpuid(7) + shr(ebx, CPUID7_EBX.FPDP) + and(ebx, 1);
    auto idavx2() => cpuid(7) + shr(ebx, CPUID7_EBX.AVX2) + and(ebx, 1);
    auto idhle() => cpuid(7) + shr(ebx, CPUID7_EBX.HLE) + and(ebx, 1);
    auto idbmi1() => cpuid(7) + shr(ebx, CPUID7_EBX.BMI1) + and(ebx, 1);
    auto idsgx() => cpuid(7) + shr(ebx, CPUID7_EBX.SGX) + and(ebx, 1);
    auto idtscadj() => cpuid(7) + shr(ebx, CPUID7_EBX.TSC_ADJUST) + and(ebx, 1);
    auto idfsgsbase() => cpuid(7) + shr(ebx, CPUID7_EBX.FSGSBASE) + and(ebx, 1);

    auto idprefetchwt1() => cpuid(7) + shr(ecx, CPUID7_ECX.PREFETCHWT1) + and(ecx, 1);
    auto idavx512vbmi() => cpuid(7) + shr(ecx, CPUID7_ECX.AVX512VBMI) + and(ecx, 1);
    auto idumip() => cpuid(7) + shr(ecx, CPUID7_ECX.UMIP) + and(ecx, 1);
    auto idpku() => cpuid(7) + shr(ecx, CPUID7_ECX.PKU) + and(ecx, 1);
    auto idospke() => cpuid(7) + shr(ecx, CPUID7_ECX.OSPKE) + and(ecx, 1);
    auto idavx512vbmi2() => cpuid(7) + shr(ecx, CPUID7_ECX.AVX512VBMI2) + and(ecx, 1);
    auto idcet() => cpuid(7) + shr(ecx, CPUID7_ECX.CET) + and(ecx, 1);
    auto idgfni() => cpuid(7) + shr(ecx, CPUID7_ECX.GFNI) + and(ecx, 1);
    auto idvaes() => cpuid(7) + shr(ecx, CPUID7_ECX.VAES) + and(ecx, 1);
    auto idvpcl() => cpuid(7) + shr(ecx, CPUID7_ECX.VPCL) + and(ecx, 1);
    auto idavx512vnni() => cpuid(7) + shr(ecx, CPUID7_ECX.AVX512VNNI) + and(ecx, 1);
    auto idavx512bitalg() => cpuid(7) + shr(ecx, CPUID7_ECX.AVX512BITALG) + and(ecx, 1);
    auto idtme() => cpuid(7) + shr(ecx, CPUID7_ECX.TME) + and(ecx, 1);
    auto idavx512vp() => cpuid(7) + shr(ecx, CPUID7_ECX.AVX512VP) + and(ecx, 1);
    auto idva57() => cpuid(7) + shr(ecx, CPUID7_ECX.VA57) + and(ecx, 1);
    auto idrdpid() => cpuid(7) + shr(ecx, CPUID7_ECX.RDPID) + and(ecx, 1);
    auto idsgxlc() => cpuid(7) + shr(ecx, CPUID7_ECX.SGX_LC) + and(ecx, 1);

    auto idavx512qvnniw() => cpuid(7) + shr(edx, CPUID7_EDX.AVX512QVNNIW) + and(edx, 1);
    auto idavx512qfma() => cpuid(7) + shr(edx, CPUID7_EDX.AVX512QFMA) + and(edx, 1);
    auto idpconfig() => cpuid(7) + shr(edx, CPUID7_EDX.PCONFIG) + and(edx, 1);
    auto idibrsibpb() => cpuid(7) + shr(edx, CPUID7_EDX.IBRS_IBPB) + and(edx, 1);
    auto idstibp() => cpuid(7) + shr(edx, CPUID7_EDX.STIBP) + and(edx, 1);

    auto idsse3() => cpuid(1) + shr(ecx, CPUID1_ECX.SSE3) + and(ecx, 1);
    auto idpclmul() => cpuid(1) + shr(ecx, CPUID1_ECX.PCLMUL) + and(ecx, 1);
    auto iddtes64() => cpuid(1) + shr(ecx, CPUID1_ECX.DTES64) + and(ecx, 1);
    auto idmon() => cpuid(1) + shr(ecx, CPUID1_ECX.MON) + and(ecx, 1);
    auto iddscpl() => cpuid(1) + shr(ecx, CPUID1_ECX.DSCPL) + and(ecx, 1);
    auto idvmx() => cpuid(1) + shr(ecx, CPUID1_ECX.VMX) + and(ecx, 1);
    auto idsmx() => cpuid(1) + shr(ecx, CPUID1_ECX.SMX) + and(ecx, 1);
    auto idest() => cpuid(1) + shr(ecx, CPUID1_ECX.EST) + and(ecx, 1);
    auto idtm2() => cpuid(1) + shr(ecx, CPUID1_ECX.TM2) + and(ecx, 1);
    auto idssse3() => cpuid(1) + shr(ecx, CPUID1_ECX.SSSE3) + and(ecx, 1);
    auto idcid() => cpuid(1) + shr(ecx, CPUID1_ECX.CID) + and(ecx, 1);
    auto idsdbg() => cpuid(1) + shr(ecx, CPUID1_ECX.SDBG) + and(ecx, 1);
    auto idfma() => cpuid(1) + shr(ecx, CPUID1_ECX.FMA) + and(ecx, 1);
    auto idcx16() => cpuid(1) + shr(ecx, CPUID1_ECX.CX16) + and(ecx, 1);
    auto idxtpr() => cpuid(1) + shr(ecx, CPUID1_ECX.XTPR) + and(ecx, 1);
    auto idpdcm() => cpuid(1) + shr(ecx, CPUID1_ECX.PDCM) + and(ecx, 1);
    auto idpcid() => cpuid(1) + shr(ecx, CPUID1_ECX.PCID) + and(ecx, 1);
    auto iddca() => cpuid(1) + shr(ecx, CPUID1_ECX.DCA) + and(ecx, 1);
    auto idsse41() => cpuid(1) + shr(ecx, CPUID1_ECX.SSE4_1) + and(ecx, 1);
    auto idsse42() => cpuid(1) + shr(ecx, CPUID1_ECX.SSE4_2) + and(ecx, 1);
    auto idx2apic() => cpuid(1) + shr(ecx, CPUID1_ECX.X2APIC) + and(ecx, 1);
    auto idmovbe() => cpuid(1) + shr(ecx, CPUID1_ECX.MOVBE) + and(ecx, 1);
    auto idpopcnt() => cpuid(1) + shr(ecx, CPUID1_ECX.POPCNT) + and(ecx, 1);
    auto idtscd() => cpuid(1) + shr(ecx, CPUID1_ECX.TSCD) + and(ecx, 1);
    auto idaes() => cpuid(1) + shr(ecx, CPUID1_ECX.AES) + and(ecx, 1);
    auto idxsave() => cpuid(1) + shr(ecx, CPUID1_ECX.XSAVE) + and(ecx, 1);
    auto idosxsave() => cpuid(1) + shr(ecx, CPUID1_ECX.OSXSAVE) + and(ecx, 1);
    auto idavx() => cpuid(1) + shr(ecx, CPUID1_ECX.AVX) + and(ecx, 1);
    auto idf16c() => cpuid(1) + shr(ecx, CPUID1_ECX.F16C) + and(ecx, 1);
    auto idrdrand() => cpuid(1) + shr(ecx, CPUID1_ECX.RDRAND) + and(ecx, 1);
    auto idhv() => cpuid(1) + shr(ecx, CPUID1_ECX.HV) + and(ecx, 1);

    auto idfpu() => cpuid(1) + shr(edx, CPUID1_EDX.FPU) + and(edx, 1);
    auto idvme() => cpuid(1) + shr(edx, CPUID1_EDX.VME) + and(edx, 1);
    auto idde() => cpuid(1) + shr(edx, CPUID1_EDX.DE) + and(edx, 1);
    auto idpse() => cpuid(1) + shr(edx, CPUID1_EDX.PSE) + and(edx, 1);
    auto idtsc() => cpuid(1) + shr(edx, CPUID1_EDX.TSC) + and(edx, 1);
    auto idmsr() => cpuid(1) + shr(edx, CPUID1_EDX.MSR) + and(edx, 1);
    auto idpae() => cpuid(1) + shr(edx, CPUID1_EDX.PAE) + and(edx, 1);
    auto idcx8() => cpuid(1) + shr(edx, CPUID1_EDX.CX8) + and(edx, 1);
    auto idapic() => cpuid(1) + shr(edx, CPUID1_EDX.APIC) + and(edx, 1);
    auto idsep() => cpuid(1) + shr(edx, CPUID1_EDX.SEP) + and(edx, 1);
    auto idmtrr() => cpuid(1) + shr(edx, CPUID1_EDX.MTRR) + and(edx, 1);
    auto idpge() => cpuid(1) + shr(edx, CPUID1_EDX.PGE) + and(edx, 1);
    auto idmca() => cpuid(1) + shr(edx, CPUID1_EDX.MCA) + and(edx, 1);
    auto idcmov() => cpuid(1) + shr(edx, CPUID1_EDX.CMOV) + and(edx, 1);
    auto idpat() => cpuid(1) + shr(edx, CPUID1_EDX.PAT) + and(edx, 1);
    auto idpse36() => cpuid(1) + shr(edx, CPUID1_EDX.PSE36) + and(edx, 1);
    auto idpsn() => cpuid(1) + shr(edx, CPUID1_EDX.PSN) + and(edx, 1);
    auto idclfl() => cpuid(1) + shr(edx, CPUID1_EDX.CLFL) + and(edx, 1);
    auto idds() => cpuid(1) + shr(edx, CPUID1_EDX.DS) + and(edx, 1);
    auto idacpi() => cpuid(1) + shr(edx, CPUID1_EDX.ACPI) + and(edx, 1);
    auto idmmx() => cpuid(1) + shr(edx, CPUID1_EDX.MMX) + and(edx, 1);
    auto idfxsr() => cpuid(1) + shr(edx, CPUID1_EDX.FXSR) + and(edx, 1);
    auto idsse() => cpuid(1) + shr(edx, CPUID1_EDX.NP) + and(edx, 1);
    auto idsse2() => cpuid(1) + shr(edx, CPUID1_EDX.SSE2) + and(edx, 1);
    auto idss() => cpuid(1) + shr(edx, CPUID1_EDX.SS) + and(edx, 1);
    auto idhtt() => cpuid(1) + shr(edx, CPUID1_EDX.HTT) + and(edx, 1);
    auto idtm() => cpuid(1) + shr(edx, CPUID1_EDX.TM) + and(edx, 1);
    auto idia64() => cpuid(1) + shr(edx, CPUID1_EDX.IA64) + and(edx, 1);
    auto idpbe() => cpuid(1) + shr(edx, CPUID1_EDX.PBE) + and(edx, 1);

    /* ====== 3DNow! ====== */
    // This is an AMD exclusive vector instruction set that uses MM registers.
    // It has been deprecated and sucks, do not use this for any kind of compiler generation.

    @("r64", "rm64")
    auto pfadd(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x9e);
    @("r64vrm64")
    auto pfsub(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x9a);
    @("r64", "rm64")
    auto pfsubr(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xaa);
    @("r64", "rm64")
    auto pfmul(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xb4);

    @("r64", "rm64")
    auto pfcmpeq(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xb0);
    @("r64", "rm64")
    auto pfcmpge(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x90);
    @("r64", "rm64")
    auto pfcmpgt(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xa0);

    @("r64", "rm64")
    auto pf2id(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x1d);
    @("r64", "rm64")
    auto pi2fd(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x0d);
    @("r64", "rm64")
    auto pf2iw(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x1c);
    @("r64", "rm64")
    auto pi2fw(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x0c);

    @("r64", "rm64")
    auto pfmax(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xa4);
    @("r64", "rm64")
    auto pfmin(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x94);

    @("r64", "rm64")
    auto pfrcp(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x96);
    @("r64", "rm64")
    auto pfrsqrt(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x97);
    @("r64", "rm64")
    auto pfrcpit1(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xa6);
    @("r64", "rm64")
    auto pfrsqit1(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xa7);
    @("r64", "rm64")
    auto pfrcpit2(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xb6);

    @("r64", "rm64")
    auto pfacc(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xae);
    @("r64", "rm64")
    auto pfnacc(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x8a);
    @("r64", "rm64")
    auto pfpnacc(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x8e);
    @("r64", "rm64")
    auto pmulhrw(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xb7);

    @("r64", "rm64")
    auto pavgusb(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xbf);
    @("r64", "rm64")
    auto pswapd(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xbb);

    auto femms() => emit!0(0x0f, 0x0e);
     
    /* ====== ICEBP ====== */
    // Intel exclusive interrupt instruction.

    auto icebp() => emit!0(0xf1);

    /* ====== PT ====== */

    @("rm32")
    auto ptwrite(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0xf3, 0x0f, 0xae, dst);
    @("rm64")
    auto ptwrite(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0xf3, 0x0f, 0xae, dst);

    /* ====== CLWB ====== */
    
    @("rm8")
    auto clwb(RM)(RM dst) if (valid!(RM, 8)) => emit!6(0x66, 0x0f, 0xae, dst);

    /* ====== CLFLUSHOPT ====== */
    
    @("rm8")
    auto clflushopt(RM)(RM dst) if (valid!(RM, 8)) => emit!7(0x66, 0x0f, 0xae, dst);

    /* ====== SMAP ====== */

    auto stac() => emit!0(0x0f, 0x01, 0xcb);
    auto clac() => emit!0(0x0f, 0x01, 0xca);

    /* ====== ADX ====== */

    @("imm8")
    auto adc(ubyte imm8) => emit!0(0x14, imm8);
    @("imm16")
    auto adc(ushort imm16) => emit!0(0x15, imm16);
    @("imm32")
    auto adc(uint imm32) => emit!0(0x15, imm32);

    @("rm8", "imm8")
    auto adc(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!2(0x80, dst, imm8);
    @("rm16", "imm16")
    auto adc(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!2(0x81, dst, imm16);
    @("rm32", "imm32")
    auto adc(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!2(0x81, dst, imm32);
    @("rm64", "imm32")
    auto adc(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!2(0x81, dst, imm32);
    @("rm16", "imm8")
    auto adc(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!2(0x83, dst, imm8);
    @("rm32", "imm8")
    auto adc(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!2(0x83, dst, imm8);
    @("rm64", "imm8")
    auto adc(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!2(0x83, dst, imm8);

    @("rm8", "r8")
    auto adc(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x10, dst, src);
    @("rm16", "r16")
    auto adc(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x11, dst, src);
    @("rm32", "r32")
    auto adc(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x11, dst, src);
    @("rm64", "r64")
    auto adc(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x11, dst, src);

    @("r8", "m8")
    auto adc(R8 dst, Mem!8 src) => emit!0(0x12, dst, src);
    @("r16", "m16")
    auto adc(R16 dst, Mem!16 src) => emit!0(0x13, dst, src);
    @("r32", "m32")
    auto adc(R32 dst, Mem!32 src) => emit!0(0x13, dst, src);
    @("r64", "m64")
    auto adc(R64 dst, Mem!64 src) => emit!0(0x13, dst, src);

    @("r32", "rm32")
    auto adcx(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0F, 0x38, 0xF6, dst, src);
    @("r64", "rm64")
    auto adcx(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0F, 0x38, 0xF6, dst, src);

    @("r32", "rm32")
    auto adox(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xF3, 0x0F, 0x38, 0xF6, dst, src);
    @("r64", "rm64")
    auto adox(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xF3, 0x0F, 0x38, 0xF6, dst, src);

    /* ====== RDSEED ====== */
    
    @("r16")
    auto rdseed(R16 dst) => emit!7(0x0f, 0xc7, dst);
    @("r32")
    auto rdseed(R32 dst) => emit!7(0x0f, 0xc7, dst);
    @("r64")
    auto rdseed(R64 dst) => emit!7(0x0f, 0xc7, dst);

    /* ====== MPX ====== */

    @("r32", "rm32")
    auto bndcl(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0x1a, dst, src);
    @("r64", "rm64")
    auto bndcl(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0x1a, dst, src);

    @("r32", "rm32")
    auto bndcu(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf2, 0x0f, 0x1a, dst, src);
    @("r64", "rm64")
    auto bndcu(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf2, 0x0f, 0x1a, dst, src);

    @("r32", "rm32")
    auto bndcn(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf2, 0x0f, 0x1b, dst, src);
    @("r64", "rm64")
    auto bndcn(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf2, 0x0f, 0x1b, dst, src);

    @("r64", "rm64")
    auto bndldx(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!(0, NP)(0x0f, 0x1a, dst, src);
    @("rm64", "r64")
    auto bndstx(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!(0, NP)(0x0f, 0x1b, dst, src);

    @("r32", "rm32")
    auto bndmk(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0x1b, dst, src);
    @("r64", "rm32")
    auto bndmk(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0x1b, dst, src);

    @("r32", "rm32")
    auto bndmov(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x1a, dst, src);
    @("r64", "rm64")
    auto bndmov(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x1a, dst, src);
    @("m32", "r32")
    auto bndmov(Mem!32 dst, R32 src) => emit!0(0x0f, 0x1b, dst, src);
    @("m64", "r32")
    auto bndmov(Mem!64 dst, R32 src) => emit!0(0x0f, 0x1b, dst, src);

    @("r16", "rm16")
    auto bound(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x62, dst, src);
    @("r32", "rm32")
    auto bound(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x62, dst, src);

    /* ====== RTM ====== */
    
    auto xend() => emit!0(0x0f, 0x01, 0xd5);
    @("imm8")
    auto xabort(ubyte imm8) => emit!0(0xc6, 0xf8, imm8);
    @("imm16")
    auto xbegin(ushort rel16) => emit!0(0xc7, 0xf8, rel16);
    @("imm32")
    auto xbegin(uint rel32) => emit!0(0xc7, 0xf8, rel32);
    auto xtest() => emit!0(0x0f, 0x01, 0xd6);
    
    /* ====== INVPCID ====== */

    @("r32", "m128")
    auto invpcid(R32 dst, Mem!128 src) => emit!0(0x0f, 0x38, 0x82, dst, src);
    @("r64", "m128")
    auto invpcid(R64 dst, Mem!128 src) => emit!0(0x0f, 0x38, 0x82, dst, src);

    /* ====== HLE ====== */

    @("prefix")
    auto xacquire(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    @("prefix")
    auto xacquire_lock(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~0xf0~buffer[(buffer.length - size)..$];
        return size + 2;
    }

    @("prefix")
    auto xrelease(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    /* ====== BMI1 ====== */

    @("r16", "rm16")
    auto tzcnt(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0xf3, 0x0f, 0xbc, dst, src);
    @("r32", "rm32")
    auto tzcnt(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0xbc, dst, src);
    @("r64", "rm64")
    auto tzcnt(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0xbc, dst, src);

    @("r16", "rm16")
    auto lzcnt(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0xf3, 0x0f, 0xbd, dst, src);
    @("r32", "rm32")
    auto lzcnt(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0xbd, dst, src);
    @("r64", "rm64")
    auto lzcnt(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0xbd, dst, src);

    @("r32", "r32", "rm32")
    auto andn(RM)(R32 dst, R32 src, RM stor) if (valid!(RM, 32)) => emit!(0, VEXI, 128, F38, 0)(0xf2, dst, src, stor);
    @("r64", "r64", "rm64")
    auto andn(RM)(R64 dst, R64 src, RM stor) if (valid!(RM, 64)) => emit!(0, VEXI, 128, F38, 0)(0xf2, dst, src, stor);

    /* ====== SGX ====== */

    auto encls() => emit!0(0x0f, 0x01, 0xcf);

    auto encls_ecreate() => mov(eax, 0) + encls();
    auto encls_eadd() => mov(eax, 1) + encls();
    auto encls_einit() => mov(eax, 2) + encls();
    auto encls_eremove() => mov(eax, 3) + encls();
    auto encls_edbgrd() => mov(eax, 4) + encls();
    auto encls_edbgwr() => mov(eax, 5) + encls();
    auto encls_eextend() => mov(eax, 6) + encls();
    auto encls_eldb() => mov(eax, 7) + encls();
    auto encls_eldu() => mov(eax, 8) + encls();
    auto encls_eblock() => mov(eax, 9) + encls();
    auto encls_epa() => mov(eax, 0xa) + encls();
    auto encls_ewb() => mov(eax, 0xb) + encls();
    auto encls_etrack() => mov(eax, 0xc) + encls();
    auto encls_eaug() => mov(eax, 0xd) + encls();
    auto encls_emodpr() => mov(eax, 0xe) + encls();
    auto encls_emodt() => mov(eax, 0xf) + encls();
    auto encls_erdinfo() => mov(eax, 0x10) + encls();
    auto encls_etrackc() => mov(eax, 0x11) + encls();
    auto encls_eldbc() => mov(eax, 0x12) + encls();
    auto encls_elduc() => mov(eax, 0x13) + encls();

    auto enclu() => emit!0(0x0f, 0x01, 0xd7);

    auto enclu_ereport() => mov(eax, 0) + enclu();
    auto enclu_egetkey() => mov(eax, 1) + enclu();
    auto enclu_eenter() => mov(eax, 2) + enclu();
    auto enclu_eresume() => mov(eax, 3) + enclu();
    auto enclu_eexit() => mov(eax, 4) + enclu();
    auto enclu_eaccept() => mov(eax, 5) + enclu();
    auto enclu_emodpe() => mov(eax, 6) + enclu();
    auto enclu_eacceptcopy() => mov(eax, 7) + enclu();
    auto enclu_edeccssa() => mov(eax, 9) + enclu();

    auto enclv() => emit!0(0x0f, 0x01, 0xc0);

    auto enclv_edecvirtchild() => mov(eax, 0) + enclv();
    auto enclv_eincvirtchild() => mov(eax, 1) + enclv();
    auto enclv_esetcontext() => mov(eax, 2) + enclv();

    /* ====== MON ====== */
    
    auto monitor() => emit!0(0x0f, 0x01, 0xc8);
    auto mwait() => emit!0(0x0f, 0x01, 0xc9);

    /* ====== VMX ====== */

    @("r32", "m128")
    auto invvpid(R32 dst, Mem!128 src) => emit!0(0x66, 0x0f, 0x38, 0x81, dst, src);
    @("r64", "m128")
    auto invvpid(R64 dst, Mem!128 src) => emit!0(0x66, 0x0f, 0x38, 0x81, dst, src);
    @("r32", "m128")
    auto invept(R32 dst, Mem!128 src) => emit!0(0x66, 0x0f, 0x38, 0x80, dst, src);
    @("r64", "m128")
    auto invept(R64 dst, Mem!128 src) => emit!0(0x66, 0x0f, 0x38, 0x80, dst, src);

    auto vmcall() => emit!0(0x0f, 0x01, 0xc1);
    auto vmfunc() => emit!0(0x0f, 0x01, 0xd4);
    @("rm64")
    auto vmclear(RM)(RM dst) if (valid!(RM, 64)) => emit!6(0x66, 0x0f, 0xc7, dst);
    auto vmlaunch() => emit!0(0x0f, 0x01, 0xc2);
    auto vmresume() => emit!0(0x0f, 0x01, 0xc3);
    auto vmxoff() => emit!0(0x0f, 0x01, 0xc4);
    @("rm64")
    auto vmxon(RM)(RM dst) if (valid!(RM, 64)) => emit!6(0xf3, 0x0f, 0xc7, dst);
    
    @("r32", "rm32")
    auto vmwrite(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!(0, NP)(0x0f, 0x79, dst, src);
    @("r64", "rm64")
    auto vmwrite(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!(0, NP)(0x0f, 0x79, dst, src);
    @("rm32", "r32")
    auto vmread(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!(0, NP)(0x0f, 0x78, dst, src);
    @("rm64", "r64")
    auto vmread(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!(0, NP)(0x0f, 0x78, dst, src);

    @("rm64")
    auto vmptrst(RM)(RM dst) if (valid!(RM, 64)) => emit!(7, NP)(0x0f, 0xc7, dst);
    @("rm64")
    auto vmptrld(RM)(RM dst) if (valid!(RM, 64)) => emit!(6, NP)(0x0f, 0xc7, dst);

    /* ====== SMX ====== */

    auto getsec() => emit!0(0x0f, 0x37);

    auto getsec_capabilities() => mov(eax, 0) + getsec();
    auto getsec_enteraccs() => mov(eax, 2) + getsec();
    auto getsec_exitac() => mov(eax, 3) + getsec();
    auto getsec_senter() => mov(eax, 4) + getsec();
    auto getsec_sexit() => mov(eax, 5) + getsec();
    auto getsec_parameters() => mov(eax, 6) + getsec();
    auto getsec_smctrl() => mov(eax, 7) + getsec();
    auto getsec_wakeup() => mov(eax, 8) + getsec();

    /* ====== CX16 ====== */

    @("m128")
    auto cmpxchg16b(Mem!128 dst) => emit!1(0x48, 0x0f, 0xc7, dst);

    /* ====== POPCNT ====== */
    
    @("r16", "rm16")
    auto popcnt(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0xf3, 0x0f, 0xb8, dst, src);
    @("r32", "rm32")
    auto popcnt(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0xb8, dst, src);
    @("r64", "rm64")
    auto popcnt(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0xb8, dst, src);
    
    /* ====== XSAVE ====== */
    
    auto xgetbv() => emit!0(0x0f, 0x01, 0xd0);
    auto xsetbv() => emit!0(0x0f, 0x01, 0xd1);

    @("m")
    auto xrstor(RM)(RM dst) if (isInstanceOf!(Mem, RM)) => emit!(5, NP)(0x0f, 0xae, dst);
    @("m")
    auto xsave(RM)(RM dst) if (isInstanceOf!(Mem, RM)) => emit!(4, NP)(0x0f, 0xae, dst);

    @("m")
    auto xrstors(RM)(RM dst) if (isInstanceOf!(Mem, RM)) => emit!(3, NP)(0x0f, 0xc7, dst);
    @("m")
    auto xsaves(RM)(RM dst) if (isInstanceOf!(Mem, RM)) => emit!(5, NP)(0x0f, 0xc7, dst);

    @("m")
    auto xsaveopt(RM)(RM dst) if (isInstanceOf!(Mem, RM)) => emit!(6, NP)(0x0f, 0xae, dst);
    @("m")
    auto xsavec(RM)(RM dst) if (isInstanceOf!(Mem, RM)) => emit!(4, NP)(0x0f, 0xc7, dst);

    /* ====== RDRAND ====== */

    @("r16")
    auto rdrand(R16 dst) => emit!6(0x0f, 0xc7, dst);
    @("r32")
    auto rdrand(R32 dst) => emit!6(0x0f, 0xc7, dst);
    @("r64")
    auto rdrand(R64 dst) => emit!6(0x0f, 0xc7, dst);

    /* ====== FPU ====== */

    auto fabs() => emit!0(0xd9, 0xe1);
    auto fchs() => emit!0(0xd9, 0xe0);

    auto fclex() => emit!0(0x9b, 0xdb, 0xe2);
    auto fnclex() => emit!0(0xdb, 0xe2);

    @("m32")
    auto fadd(Mem!32 dst) => emit!(0, NP)(0xd8, dst);
    @("m64")
    auto fadd(Mem!64 dst) => emit!(0, NP)(0xdc, dst);
    @("st", "st")
    auto fadd(ST dst, ST src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xc0, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xc0, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    @("st")
    auto faddp(ST dst) => emit!(0, NRM)(0xde, 0xc0, dst);
    @("m16")
    auto fiadd(Mem!16 dst) => emit!(0, NP)(0xde, dst);
    @("m32")
    auto fiadd(Mem!32 dst) => emit!(0, NP)(0xda, dst);

    @("m80")
    auto fbld(Mem!80 dst) => emit!(4, NP)(0xdf, dst);
    @("m80")
    auto fbstp(Mem!80 dst) => emit!(6, NP)(0xdf, dst);

    @("m32")
    auto fcom(Mem!32 dst) => emit!(2, NP)(0xd8, dst);
    @("m64")
    auto fcom(Mem!64 dst) => emit!(2, NP)(0xdc, dst);
    @("st")
    auto fcom(ST dst) => emit!(2, NRM)(0xd8, 0xd0, dst);

    @("m32")
    auto fcomp(Mem!32 dst) => emit!(3, NP)(0xd8, dst);
    @("m64")
    auto fcomp(Mem!64 dst) => emit!(3, NP)(0xdc, dst);
    @("st")
    auto fcomp(ST dst) => emit!(2, NRM)(0xd8, 0xd8, dst);
    auto fcompp() => emit!0(0xde, 0xd9);

    @("st")
    auto fcomi(ST dst) => emit!(0, NRM)(0xdb, 0xf0, dst);
    @("st")
    auto fcomip(ST dst) => emit!(0, NRM)(0xdf, 0xf0, dst);
    @("st")
    auto fucomi(ST dst) => emit!(0, NRM)(0xdb, 0xe8, dst);
    @("st")
    auto fucomip(ST dst) => emit!(0, NRM)(0xdf, 0xe8, dst);

    @("m16")
    auto ficom(Mem!16 dst) => emit!(2, NP)(0xde, dst);
    @("m32")
    auto ficom(Mem!32 dst) => emit!(2, NP)(0xda, dst);
    @("m16")
    auto ficomp(Mem!16 dst) => emit!(2, NP)(0xde, dst);
    @("m32")
    auto ficomp(Mem!32 dst) => emit!(2, NP)(0xda, dst);
    
    @("st")
    auto fucom(ST dst) => emit!(2, NRM)(0xdd, 0xe0, dst);
    @("st")
    auto fucomp(ST dst) => emit!(2, NRM)(0xdd, 0xe8, dst);
    auto fucompp() => emit!0(0xda, 0xe9);

    auto ftst() => emit!0(0xd9, 0xe4);

    auto f2xm1() => emit!0(0xd9, 0xf0);
    auto fyl2x() => emit!0(0xd9, 0xf1);
    auto fyl2xp1() => emit!0(0xd9, 0xf9);

    auto fcos() => emit!0(0xd9, 0xff);
    auto fsin() => emit!0(0xd9, 0xfe);
    auto fsincos() => emit!0(0xd9, 0xfb);
    auto fsqrt() => emit!0(0xd9, 0xfa);
    
    auto fptan() => emit!0(0xd9, 0xf2);
    auto fpatan() => emit!0(0xd9, 0xf3);
    auto fprem() => emit!0(0xd9, 0xf8);
    auto fprem1() => emit!0(0xd9, 0xf5);

    auto fdecstp() => emit!0(0xd9, 0xf6);
    auto fincstp() => emit!0(0xd9, 0xf7);

    @("m16")
    auto fild(Mem!16 dst) => emit!(0, NP)(0xdf, dst);
    @("m32")
    auto fild(Mem!32 dst) => emit!(0, NP)(0xdb, dst);
    @("m64")
    auto fild(Mem!64 dst) => emit!(5, NP)(0xdf, dst);

    @("m16")
    auto fist(Mem!16 dst) => emit!(2, NP)(0xdf, dst);
    @("m32")
    auto fist(Mem!32 dst) => emit!(2, NP)(0xdb, dst);

    @("m16")
    auto fistp(Mem!16 dst) => emit!(3, NP)(0xdf, dst);
    @("m32")
    auto fistp(Mem!32 dst) => emit!(3, NP)(0xdb, dst);
    @("m64")
    auto fistp(Mem!64 dst) => emit!(7, NP)(0xdf, dst);

    @("m16")
    auto fisttp(Mem!16 dst) => emit!(1, NP)(0xdf, dst);
    @("m32")
    auto fisttp(Mem!32 dst) => emit!(1, NP)(0xdb, dst);
    @("m64")
    auto fisttp(Mem!64 dst) => emit!(1, NP)(0xdd, dst);

    @("m16")
    auto fldcw(Mem!16 dst) => emit!(5, NP)(0xd9, dst);
    @("m16")
    auto fstcw(Mem!16 dst) => emit!(7, NP)(0x9b, 0xd9, dst);
    @("m16")
    auto fnstcw(Mem!16 dst) => emit!(7, NP)(0xd9, dst);

    @("m112")
    auto fldenv(Mem!112 dst) => emit!(4, NP)(0xd9, dst);
    @("m224")
    auto fldenv(Mem!224 dst) => emit!(4, NP)(0xd9, dst);

    @("m112")
    auto fstenv(Mem!112 dst) => emit!(6, NP)(0x9b, 0xd9, dst);
    @("m224")
    auto fstenv(Mem!224 dst) => emit!(6, NP)(0x9b, 0xd9, dst);

    @("m112")
    auto fnstenv(Mem!112 dst) => emit!(6, NP)(0xd9, dst);
    @("m224")
    auto fnstenv(Mem!224 dst) => emit!(6, NP)(0xd9, dst);

    @("m16")
    auto fstsw(Mem!16 dst) => emit!(7, NP)(0x9b, 0xdd, dst);
    auto fstsw() => emit!0(0x9b, 0xdf, 0xe0);
    @("m16")
    auto fnstsw(Mem!16 dst) => emit!(7, NP)(0xdd, dst);
    auto fnstsw() => emit!0(0xdf, 0xe0);

    @("m32")
    auto fld(Mem!32 dst) => emit!(0, NP)(0xd9, dst);
    @("m64")
    auto fld(Mem!64 dst) => emit!(0, NP)(0xdd, dst);
    @("m80")
    auto fld(Mem!80 dst) => emit!(5, NP)(0xdb, dst);
    @("st")
    auto fld(ST dst) => emit!(0, NRM)(0xd9, 0xc0, dst);

    auto fld1() => emit!0(0xd9, 0xe8);
    auto fldl2t() => emit!0(0xd9, 0xe9);
    auto fldl2e() => emit!0(0xd9, 0xea);
    auto fldpi() => emit!0(0xd9, 0xeb);
    auto fldlg2() => emit!0(0xd9, 0xec);
    auto fldln2() => emit!0(0xd9, 0xed);
    auto fldz() => emit!0(0xd9, 0xee);

    @("m32")
    auto fst(Mem!32 dst) => emit!(2, NP)(0xd9, dst);
    @("m64")
    auto fst(Mem!64 dst) => emit!(2, NP)(0xdd, dst);
    @("st")
    auto fst(ST dst) => emit!(0, NRM)(0xdd, 0xd0, dst);
    
    @("m32")
    auto fstp(Mem!32 dst) => emit!(3, NP)(0xd9, dst);
    @("m64")
    auto fstp(Mem!64 dst) => emit!(3, NP)(0xdd, dst);
    @("m80")
    auto fstp(Mem!80 dst) => emit!(7, NP)(0xdb, dst);
    @("st")
    auto fstp(ST dst) => emit!(0, NRM)(0xdd, 0xd8, dst);

    @("m32")
    auto fdiv(Mem!32 dst) => emit!(6, NP)(0xd8, dst);
    @("m64")
    auto fdiv(Mem!64 dst) => emit!(6, NP)(0xdc, dst);
    @("st", "st")
    auto fdiv(ST dst, ST src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xf0, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xf8, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    @("st")
    auto fdivp(ST dst) => emit!(0, NRM)(0xde, 0xf8, dst);
    auto fidiv(Mem!16 dst) => emit!(6, NP)(0xde, dst);
    @("m32")
    auto fidiv(Mem!32 dst) => emit!(6, NP)(0xda, dst);

    @("m32")
    auto fdivr(Mem!32 dst) => emit!(7, NP)(0xd8, dst);
    @("m64")
    auto fdivr(Mem!64 dst) => emit!(7, NP)(0xdc, dst);
    @("st", "st")
    auto fdivr(ST dst, ST src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xf8, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xf0, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    @("st")
    auto fdivrp(ST dst) => emit!(0, NRM)(0xde, 0xf0, dst);
    @("m16")
    auto fidivr(Mem!16 dst) => emit!(7, NP)(0xde, dst);
    @("m32")
    auto fidivr(Mem!32 dst) => emit!(7, NP)(0xda, dst);

    auto fscale() => emit!0(0xd9, 0xfd);
    auto frndint() => emit!0(0xd9, 0xfc);
    auto fexam() => emit!0(0xd9, 0xe5);
    @("st")
    auto ffree(ST dst) => emit!(0, NRM)(0xdd, 0xc0, dst);
    @("st")
    auto fxch(ST dst) => emit!(0, NRM)(0xd9, 0xc8, dst);
    auto fxtract() => emit!0(0xd9, 0xf4);

    auto fnop() => emit!0(0xd9, 0xd0);
    auto fninit() => emit!0(0x9b, 0xdb, 0xe3);
    auto finit() => emit!0(0xdb, 0xe3);

    @("m752")
    auto fsave(Mem!752 dst) => emit!6(0x9b, 0xdd, dst);
    @("m864")
    auto fsave(Mem!864 dst) => emit!6(0x9b, 0xdd, dst);

    @("m752")
    auto fnsave(Mem!752 dst) => emit!6(0xdd, dst);
    @("m864")
    auto fnsave(Mem!864 dst) => emit!6(0xdd, dst);

    @("m752")
    auto frstor(Mem!752 dst) => emit!4(0xdd, dst);
    @("m864")
    auto frstor(Mem!864 dst) => emit!4(0xdd, dst);

    static if (!X64)
    @("m4096")
    auto fxsave(Mem!4096 dst) => emit!(0, NP)(0x0f, 0xae, dst);
    static if (X64)
    @("m4096")
    auto fxsave(Mem!4096 dst) => emit!(0, NP)(0x48, 0x0f, 0xae, dst);
    
    static if (!X64)
    @("m4096")
    auto fxrstor(Mem!4096 dst) => emit!(1, NP)(0x0f, 0xae, dst);
    static if (X64)
    @("m4096")
    auto fxrstor(Mem!4096 dst) => emit!(1, NP)(0x48, 0x0f, 0xae, dst);

    @("m32")
    auto fmul(Mem!32 dst) => emit!(1, NP)(0xd8, dst);
    @("m64")
    auto fmul(Mem!64 dst) => emit!(1, NP)(0xdc, dst);
    @("st", "st")
    auto fmul(ST dst, ST src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xc8, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xc8, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    @("st")
    auto fmulp(ST dst) => emit!(0, NRM)(0xde, 0xc8, dst);
    @("m16")
    auto fimul(Mem!16 dst) => emit!(1, NP)(0xde, dst);
    @("m32")
    auto fimul(Mem!32 dst) => emit!(1, NP)(0xda, dst);

    @("m32")
    auto fsub(Mem!32 dst) => emit!(4, NP)(0xd8, dst);
    @("m64")
    auto fsub(Mem!64 dst) => emit!(4, NP)(0xdc, dst);
    @("st", "st")
    auto fsub(ST dst, ST src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xe0, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xe8, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    @("st")
    auto fsubp(ST dst) => emit!(0, NRM)(0xde, 0xe8, dst);
    @("m16")
    auto fisub(Mem!16 dst) => emit!(4, NP)(0xde, dst);
    @("m32")
    auto fisub(Mem!32 dst) => emit!(4, NP)(0xda, dst);

    @("m32")
    auto fsubr(Mem!32 dst) => emit!(5, NP)(0xd8, dst);
    @("m64")
    auto fsubr(Mem!64 dst) => emit!(5, NP)(0xdc, dst);
    @("st", "st")
    auto fsubr(ST dst, ST src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xe8, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xe0, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    @("st")
    auto fsubrp(ST dst) => emit!(0, NRM)(0xde, 0xe0, dst);
    @("m16")
    auto fisubr(Mem!16 dst) => emit!(5, NP)(0xde, dst);
    @("m32")
    auto fisubr(Mem!32 dst) => emit!(5, NP)(0xda, dst);

    @("st")
    auto fcmovb(ST dst) => emit!(0, NRM)(0xda, 0xc0, dst);
    @("st")
    auto fcmove(ST dst) => emit!(0, NRM)(0xda, 0xc8, dst);
    @("st")
    auto fcmovbe(ST dst) => emit!(0, NRM)(0xda, 0xd0, dst);
    @("st")
    auto fcmovu(ST dst) => emit!(0, NRM)(0xda, 0xd8, dst);
    @("st")
    auto fcmovnb(ST dst) => emit!(0, NRM)(0xdb, 0xc0, dst);
    @("st")
    auto fcmovne(ST dst) => emit!(0, NRM)(0xdb, 0xc8, dst);
    @("st")
    auto fcmovnbe(ST dst) => emit!(0, NRM)(0xdb, 0xd0, dst);
    @("st")
    auto fcmovnu(ST dst) => emit!(0, NRM)(0xdb, 0xd8, dst);

    /* ====== TSC ====== */

    auto rdtsc() => emit!0(0x0f, 0x31);
    auto rdtscp() => emit!0(0x0f, 0x01, 0xf9);

    /* ====== MSR ====== */

    auto rdmsr() => emit!0(0x0f, 0x32);
    auto wrmsr() => emit!0(0x0f, 0x30);
    
    /* ====== CX8 ====== */

    @("m64")
    auto cmpxchg8b(Mem!64 dst) => emit!(1, NP)(0x0f, 0xc7, dst);

    /* ====== SEP ====== */
    
    auto sysenter() => emit!0(0x0f, 0x34);
    auto sysexitc() => emit!0(0x0f, 0x35);
    auto sysexit() => emit!0(0x0f, 0x35);

    /* ====== CMOV ====== */

    @("r16", "rm16")
    auto cmova(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x47, dst, src);
    @("r32", "rm32")
    auto cmova(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x47, dst, src);
    @("r64", "rm64")
    auto cmova(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x47, dst, src);
    
    @("r16", "rm16")
    auto cmovae(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x43, dst, src);
    @("r32", "rm32")
    auto cmovae(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x43, dst, src);
    @("r64", "rm64")
    auto cmovae(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x43, dst, src);
    
    @("r16", "rm16")
    auto cmovb(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x42, dst, src);
    @("r32", "rm32")
    auto cmovb(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x42, dst, src);
    @("r64", "rm64")
    auto cmovb(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x42, dst, src);
    
    @("r16", "rm16")
    auto cmovbe(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x46, dst, src);
    @("r32", "rm32")
    auto cmovbe(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x46, dst, src);
    @("r64", "rm64")
    auto cmovbe(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x46, dst, src);
    
    @("r16", "rm16")
    auto cmovc(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x42, dst, src);
    @("r32", "rm32")
    auto cmovc(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x42, dst, src);
    @("r64", "rm64")
    auto cmovc(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x42, dst, src);
    
    @("r16", "rm16")
    auto cmove(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x44, dst, src);
    @("r32", "rm32")
    auto cmove(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x44, dst, src);
    @("r64", "rm64")
    auto cmove(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x44, dst, src);
    
    @("r16", "rm16")
    auto cmovg(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4f, dst, src);
    @("r32", "rm32")
    auto cmovg(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4f, dst, src);
    @("r64", "rm64")
    auto cmovg(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4f, dst, src);
    
    @("r16", "rm16")
    auto cmovge(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4d, dst, src);
    @("r32", "rm32")
    auto cmovge(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4d, dst, src);
    @("r64", "rm64")
    auto cmovge(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4d, dst, src);
    
    @("r16", "rm16")
    auto cmovl(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4c, dst, src);
    @("r32", "rm32")
    auto cmovl(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4c, dst, src);
    @("r64", "rm64")
    auto cmovl(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4c, dst, src);
    
    @("r16", "rm16")
    auto cmovle(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4e, dst, src);
    @("r32", "rm32")
    auto cmovle(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4e, dst, src);
    @("r64", "rm64")
    auto cmovle(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4e, dst, src);
    
    @("r16", "rm16")
    auto cmovna(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x46, dst, src);
    @("r32", "rm32")
    auto cmovna(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x46, dst, src);
    @("r64", "rm64")
    auto cmovna(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x46, dst, src);
    
    @("r16", "rm16")
    auto cmovnae(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x42, dst, src);
    @("r32", "rm32")
    auto cmovnae(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x42, dst, src);
    @("r64", "rm64")
    auto cmovnae(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x42, dst, src);
    
    @("r16", "rm16")
    auto cmovnb(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x43, dst, src);
    @("r32", "rm32")
    auto cmovnb(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x43, dst, src);
    @("r64", "rm64")
    auto cmovnb(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x43, dst, src);
    
    @("r16", "rm16")
    auto cmovnbe(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x47, dst, src);
    @("r32", "rm32")
    auto cmovnbe(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x47, dst, src);
    @("r64", "rm64")
    auto cmovnbe(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x47, dst, src);
    
    @("r16", "rm16")
    auto cmovnc(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x43, dst, src);
    @("r32", "rm32")
    auto cmovnc(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x43, dst, src);
    @("r64", "rm64")
    auto cmovnc(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x43, dst, src);
    
    @("r16", "rm16")
    auto cmovne(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x45, dst, src);
    @("r32", "rm32")
    auto cmovne(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x45, dst, src);
    @("r64", "rm64")
    auto cmovne(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x45, dst, src);
    
    @("r16", "rm16")
    auto cmovng(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4e, dst, src);
    @("r32", "rm32")
    auto cmovng(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4e, dst, src);
    auto cmovng(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4e, dst, src);
    
    @("r16", "rm16")
    auto cmovnge(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4c, dst, src);
    @("r32", "rm32")
    auto cmovnge(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4c, dst, src);
    @("r64", "rm64")
    auto cmovnge(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4c, dst, src);
    
    @("r16", "rm16")
    auto cmovnl(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4d, dst, src);
    @("r32", "rm32")
    auto cmovnl(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4d, dst, src);
    @("r64", "rm64")
    auto cmovnl(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4d, dst, src);
    
    @("r16", "rm16")
    auto cmovnle(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4f, dst, src);
    @("r32", "rm32")
    auto cmovnle(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4f, dst, src);
    @("r64", "rm64")
    auto cmovnle(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4f, dst, src);
    
    @("r16", "rm16")
    auto cmovno(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x41, dst, src);
    @("r32", "rm32")
    auto cmovno(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x41, dst, src);
    @("r64", "rm64")
    auto cmovno(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x41, dst, src);
    
    @("r16", "rm16")
    auto cmovnp(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4b, dst, src);
    @("r32", "rm32")
    auto cmovnp(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4b, dst, src);
    @("r64", "rm64")
    auto cmovnp(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4b, dst, src);
    
    @("r16", "rm16")
    auto cmovns(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x49, dst, src);
    @("r32", "rm32")
    auto cmovns(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x49, dst, src);
    @("r64", "rm64")
    auto cmovns(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x49, dst, src);
    
    @("r16", "rm16")
    auto cmovnz(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x45, dst, src);
    @("r32", "rm32")
    auto cmovnz(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x45, dst, src);
    @("r64", "rm64")
    auto cmovnz(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x45, dst, src);
    
    @("r16", "rm16")
    auto cmovo(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x40, dst, src);
    @("r32", "rm32")
    auto cmovo(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x40, dst, src);
    @("r64", "rm64")
    auto cmovo(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x40, dst, src);
    
    @("r16", "rm16")
    auto cmovp(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4a, dst, src);
    @("r32", "rm32")
    auto cmovp(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4a, dst, src);
    @("r64", "rm64")
    auto cmovp(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4a, dst, src);
    
    @("r16", "rm16")
    auto cmovpe(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4a, dst, src);
    @("r32", "rm32")
    auto cmovpe(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4a, dst, src);
    @("r64", "rm64")
    auto cmovpe(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4a, dst, src);
    
    @("r16", "rm16")
    auto cmovpo(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4b, dst, src);
    @("r32", "rm32")
    auto cmovpo(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4b, dst, src);
    @("r64", "rm64")
    auto cmovpo(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4b, dst, src);
    
    @("r16", "rm16")
    auto cmovs(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x48, dst, src);
    @("r32", "rm32")
    auto cmovs(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x48, dst, src);
    @("r64", "rm64")
    auto cmovs(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x48, dst, src);
    
    @("r16", "rm16")
    auto cmovz(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x44, dst, src);
    @("r32", "rm32")
    auto cmovz(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x44, dst, src);
    @("r64", "rm64")
    auto cmovz(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x44, dst, src);

    /* ====== CLFL ====== */

    @("rm8")
    auto clflush(RM)(RM dst) if (valid!(RM, 8)) => emit!(7, NP)(0x0f, 0xae, dst);

    /* ====== HRESET ====== */

    @("mimm8")
    auto hreset(ubyte imm8) => emit!0(0xf3, 0x0f, 0x3a, 0xf0, 0xc0, imm8, eax);

    /* ====== CET ====== */
    // Shadow stack instruction set.

    @("r32")
    auto incsspd(R32 dst) => emit!5(0xf3, 0x0f, 0xae, dst);
    @("r64")
    auto incsspq(R64 dst) => emit!5(0xf3, 0x0f, 0xae, dst);

    @("m64")
    auto clrssbsy(Mem!64 dst) => emit!6(0xf3, 0x0f, 0xae, dst);
    auto setssbsy() => emit!0(0xf3, 0x0f, 0x01, 0xe8);

    @("r32")
    auto rdsspd(R32 dst) => emit!1(0xf3, 0x0f, 0x1e, dst);
    @("r64")
    auto rdsspq(R64 dst) => emit!1(0xf3, 0x0f, 0x1e, dst);

    @("m32", "r32")
    auto wrssd(Mem!32 dst, R32 src) => emit!0(0xf3, 0x38, 0xf6, dst, src);
    @("m64", "r64")
    auto wrssq(Mem!64 dst, R64 src) => emit!0(0xf3, 0x38, 0xf6, dst, src);

    @("m32", "r32")
    auto wrussd(Mem!32 dst, R32 src) => emit!1(0x66, 0xf3, 0x38, 0xf5, dst, src);
    @("m64", "r64")
    auto wrussq(Mem!64 dst, R64 src) => emit!1(0x66, 0xf3, 0x38, 0xf5, dst, src);

    @("m64")
    auto rstorssp(Mem!64 dst) => emit!5(0xf3, 0x0f, 0x01, dst);
    auto saveprevssp() => emit!5(0xf3, 0x0f, 0x01, 0xae, edx);

    auto endbr32() => emit!0(0xf3, 0x0f, 0x1e, 0xfb);
    auto endbr64() => emit!0(0xf3, 0x0f, 0x1e, 0xfa);

    /* ====== FSGSBASE ====== */

    @("r32")
    auto rdfsbase(R32 dst) => emit!0(0xf3, 0x0f, 0xae, dst);
    @("r64")
    auto rdfsbase(R64 dst) => emit!0(0xf3, 0x0f, 0xae, dst);

    @("r32")
    auto rdgsbase(R32 dst) => emit!1(0xf3, 0x0f, 0xae, dst);
    @("r64")
    auto rdgsbase(R64 dst) => emit!1(0xf3, 0x0f, 0xae, dst);

    @("r32")
    auto wrfsbase(R32 dst) => emit!2(0xf3, 0x0f, 0xae, dst);
    @("r64")
    auto wrfsbase(R64 dst) => emit!2(0xf3, 0x0f, 0xae, dst);
    
    @("r32")
    auto wrgsbase(R32 dst) => emit!3(0xf3, 0x0f, 0xae, dst);
    @("r64")
    auto wrgsbase(R64 dst) => emit!3(0xf3, 0x0f, 0xae, dst);

    /* ====== RDPID ====== */

    @("r32")
    auto rdpid(R32 dst) => emit!7(0xf3, 0x0f, 0xc7, dst);
    @("r64")
    auto rdpid(R64 dst) => emit!7(0xf3, 0x0f, 0xc7, dst);

    /* ====== OSPKE ====== */

    auto wrpkru() => emit!0(0x0f, 0x01, 0xef);
    auto rdpkru() => emit!0(0x0f, 0x01, 0xee);

    /* ====== UINTR ====== */

    auto testui() => emit!0(0xf3, 0x0f, 0x01, 0xed);
    auto stui() => emit!0(0xf3, 0x0f, 0x01, 0xef);
    auto clui() => emit!0(0xf3, 0x0f, 0x01, 0xee);
    auto uiret() => emit!0(0xf3, 0x0f, 0x01, 0xec);

    @("rm8")
    auto senduipi(RM)(RM dst) if (valid!(RM, 8)) => emit!6(0xf3, 0x0f, 0xc7, dst);
    @("rm16")
    auto senduipi(RM)(RM dst) if (valid!(RM, 16)) => emit!6(0xf3, 0x0f, 0xc7, dst);
    @("rm32")
    auto senduipi(RM)(RM dst) if (valid!(RM, 32)) => emit!6(0xf3, 0x0f, 0xc7, dst);
    @("rm64")
    auto senduipi(RM)(RM dst) if (valid!(RM, 64)) => emit!6(0xf3, 0x0f, 0xc7, dst);

    /* ====== WAITPKG ====== */
    
    @("r32")
    auto umwait(R32 dst) => emit!6(0xf2, 0x0f, 0xae, dst);
    @("r16")
    auto umonitor(R16 dst) => emit!6(0xf3, 0x0f, 0xae, dst);
    @("r32")
    auto umonitor(R32 dst) => emit!6(0xf3, 0x0f, 0xae, dst);
    @("r64")
    auto umonitor(R64 dst) => emit!6(0xf3, 0x0f, 0xae, dst);
    @("r32")
    auto tpause(R32 dst) => emit!6(0x0f, 0xae, dst);

    /* ====== CLDEMOTE ====== */
    
    @("rm8")
    auto cldemote(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x1c, dst);

    /* ====== TSXLDTRK ====== */

    auto xresldtrk() => emit!0(0xf2, 0x0f, 0x01, 0xe9);
    auto xsusldtrk() => emit!0(0xf2, 0x0f, 0x01, 0xe8);

    /* ====== SERALIZE ====== */
    
    auto serialize() => emit!0(0x0f, 0x01, 0xe8);

    /* ====== PCONFIG ====== */

    auto pconfig() => emit!0(0x0f, 0x01, 0xc5);

    /* ====== PMC ====== */

    auto rdpmc() => emit!0(0x0f, 0x33); 

    /* ====== UMIP ====== */

    auto wbinvd() => emit!0(0x0f, 0x09);
    auto wbnoinvd() => emit!0(0xf3, 0x0f, 0x09);
    
    auto invd() => emit!0(0x0f, 0x08);

    @("rm32")
    auto lgdt(RM)(RM dst) if (valid!(RM, 32)) => emit!2(0x0f, 0x01, dst);
    @("rm64")
    auto lgdt(RM)(RM dst) if (valid!(RM, 64)) => emit!2(0x0f, 0x01, dst);
    @("rm64")
    auto sgdt(RM)(RM dst) if (valid!(RM, 64)) => emit!0(0x0f, 0x01, dst);

    @("rm16")
    auto lldt(RM)(RM dst) if (valid!(RM, 16)) => emit!2(0x0f, 0x00, dst);
    @("rm16")
    auto sldt(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0x0f, 0x00, dst);

    @("rm32")
    auto lidt(RM)(RM dst) if (valid!(RM, 32)) => emit!3(0x0f, 0x01, dst);
    @("rm64")
    auto lidt(RM)(RM dst) if (valid!(RM, 64)) => emit!3(0x0f, 0x01, dst);
    @("rm64")
    auto sidt(RM)(RM dst) if (valid!(RM, 64)) => emit!1(0x0f, 0x01, dst);

    @("rm16")
    auto lmsw(RM)(RM dst) if (valid!(RM, 16)) => emit!6(0x0f, 0x01, dst);

    @("rm16")
    auto smsw(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0x0f, 0x01, dst);
    @("rm32")
    auto smsw(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0x0f, 0x01, dst);
    @("rm64")
    auto smsw(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0x0f, 0x01, dst);

    /* ====== PCID ====== */

    @("rm64")
    auto invlpg(RM)(RM dst) if (valid!(RM, 64)) => emit!7(0x0f, 0x01, dst);

    /* ====== LAHF-SAHF ====== */

    auto sahf() => emit!0(0x9e);
    auto lahf() => emit!0(0x9f);

    /* ====== BMI2 ====== */

    auto sarx(RM)(R32 dst, RM src, R32 cnt) if (valid!(RM, 32)) => emit!(0, VEXI, 128, F38, 0xf3)(0xf7, dst, src, cnt);
    auto shlx(RM)(R32 dst, RM src, R32 cnt) if (valid!(RM, 32)) => emit!(0, VEXI, 128, F38, 0x66)(0xf7, dst, src, cnt);
    auto shrx(RM)(R32 dst, RM src, R32 cnt) if (valid!(RM, 32)) => emit!(0, VEXI, 128, F38, 0xf2)(0xf7, dst, src, cnt);

    auto sarx(RM)(R64 dst, RM src, R64 cnt) if (valid!(RM, 64)) => emit!(0, VEXI, 128, F38, 0xf3)(0xf7, dst, src, cnt);
    auto shlx(RM)(R64 dst, RM src, R64 cnt) if (valid!(RM, 64)) => emit!(0, VEXI, 128, F38, 0x66)(0xf7, dst, src, cnt);
    auto shrx(RM)(R64 dst, RM src, R64 cnt) if (valid!(RM, 64)) => emit!(0, VEXI, 128, F38, 0xf2)(0xf7, dst, src, cnt);

    /* ====== MMX ====== */

    auto movq(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!(0, SSE)(0x0f, 0x6f, dst, src);
    auto movq(Mem!64 dst, MMX src) => emit!(0, SSE)(0x0f, 0x7f, dst, src);

    auto movd(RM)(MMX dst, RM src) if (valid!(RM, 32)) => emit!(0, SSE)(0x0f, 0x6e, dst, src);
    auto movd(RM)(RM dst, MMX src) if (valid!(RM, 32)) => emit!(0, SSE)(0x0f, 0x7e, dst, src);

    auto movq(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x6e, dst, src);
    auto movq(RM)(RM dst, MMX src) if (valid!(RM, 64)) => emit!0(0x0f, 0x7e, dst, src);

    /* ====== SSE ====== */

    auto addpd(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x58, dst, src);
    auto addps(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x58, dst, src);
    auto addss(RM)(XMM dst, RM src) if (valid!(RM, 128, 32)) => emit!(0, SSE)(0xf3, 0x0f, 0x58, dst, src);
    auto addsd(RM)(XMM dst, RM src) if (valid!(RM, 128, 32)) => emit!(0, SSE)(0xf2, 0x0f, 0x58, dst, src);

    /* ====== SSE2 ====== */

    auto lfence() => emit!0(0x0f, 0xae, 0xe8);
    auto sfence() => emit!0(0x0f, 0xae, 0xf8);
    auto mfence() => emit!0(0x0f, 0xae, 0xf0);

    auto movq(RM)(XMM dst, RM src) if (valid!(RM, 128, 64)) => emit!(0, SSE)(0xf3, 0x0f, 0x7e, dst, src);
    auto movq(Mem!64 dst, XMM src) => emit!(0, SSE)(0x66, 0x0f, 0xd6, dst, src);

    auto movd(RM)(XMM dst, RM src) if (valid!(RM, 32)) => emit!(0, SSE)(0x66, 0x0f, 0x6e, dst, src);
    auto movd(RM)(RM dst, XMM src) if (valid!(RM, 32)) => emit!(0, SSE)(0x66, 0x0f, 0x7e, dst, src);
    // TODO: This won't flip dst and src but should also also should generate a REX
    auto movq(RM)(XMM dst, RM src) if (valid!(RM, 64)) => emit!0(0x66, 0x0f, 0x6e, dst, src);
    auto movq(RM)(RM dst, XMM src) if (valid!(RM, 64)) => emit!0(0x66, 0x0f, 0x7e, dst, src);

    /* ====== SSE3 ====== */

    auto addsubps(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0xf2, 0x0f, 0xd0, dst, src);
    auto addsubpd(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0xd0, dst, src);
    
    /* ====== AVX ====== */

    auto vaddpd(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x58, dst, src, stor);
    auto vaddpd(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, DEFAULT, 0x66)(0x58, dst, src, stor);
     
    auto vaddps(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, DEFAULT, 0)(0x58, dst, src, stor);
    auto vaddps(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, DEFAULT, 0)(0x58, dst, src, stor);

    auto vaddsd(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128, 64)) => emit!(0, VEX, 128, DEFAULT, 0xf2)(0x58, dst, src, stor);
    auto vaddss(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128, 32)) => emit!(0, VEX, 128, DEFAULT, 0xf3)(0x58, dst, src, stor);

    auto vaddsubpd(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0xd0, dst, src, stor);
    auto vaddsubpd(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, DEFAULT, 0x66)(0xd0, dst, src, stor);
     
    auto vaddsubps(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, DEFAULT, 0xf2)(0xd0, dst, src, stor);
    auto vaddsubps(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, DEFAULT, 0xf2)(0xd0, dst, src, stor);

    auto vmovq(RM)(XMM dst, RM src) if (valid!(RM, 128, 64)) => emit!(0, VEX, 128, DEFAULT, 0xf3)(0x7e, dst, src);
    auto vmovq(Mem!64 dst, XMM src) => emit!(0, VEX, 128, DEFAULT, 0x66)(0xd6, dst, src);

    @("r128", "rm32")
    auto vmovd(RM)(XMM dst, RM src) if (valid!(RM, 32)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x6e, dst, src);
    auto vmovd(RM)(RM dst, XMM src) if (valid!(RM, 32)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x7e, dst, src);

    @("r128", "rm64")
    auto vmovq(RM)(XMM dst, RM src) if (valid!(RM, 64)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x6e, dst, src);
    auto vmovq(RM)(RM dst, XMM src) if (valid!(RM, 64)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x7e, dst, src);

    /* ====== AES ====== */

    @("r128", "rm128")
    auto aesdec(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xde, dst, src);
    auto vaesdec(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xde, dst, src, stor);
    auto vaesdec(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, F38, 0x66)(0xde, dst, src, stor);

    auto aesdec128kl(XMM dst, Mem!384 src) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xdd, dst, src);
    auto aesdec256kl(XMM dst, Mem!512 src) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xdf, dst, src);

    @("r128", "rm128")
    auto aesdeclast(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xdf, dst, src);
    auto vaesdeclast(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xdf, dst, src, stor);
    auto vaesdeclast(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, F38, 0x66)(0xdf, dst, src, stor);

    auto aesdecwide128kl(Mem!384 dst) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xd8, dst, ecx);
    auto aesdecwide256kl(Mem!512 dst) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xd8, dst, ebx);

    @("r128", "rm128")
    auto aesenc(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xdc, dst, src);
    auto vaesenc(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xdc, dst, src, stor);
    auto vaesenc(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, F38, 0x66)(0xdc, dst, src, stor);

    auto aesenc128kl(XMM dst, Mem!384 src) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xdc, dst, src);
    auto aesenc256kl(XMM dst, Mem!512 src) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xde, dst, src);

    @("r128", "rm128")
    auto aesenclast(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xdd, dst, src);
    auto vaesenclast(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xdd, dst, src, stor);
    auto vaesenclast(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, F38, 0x66)(0xdd, dst, src, stor);

    auto aesencwide128kl(Mem!384 dst) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xd8, dst, eax);
    auto aesencwide256kl(Mem!512 dst) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xd8, dst, edx);

    @("r128", "rm128")
    auto aesimc(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xdb, dst, src);
    @("r128", "rm128")
    auto vaesimc(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xdb, dst, src);

    @("r128", "rm128", "imm8")
    auto aeskeygenassist(RM)(XMM dst, RM src, ubyte imm8) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x3a, 0xdf, dst, src, imm8);
    @("r128", "rm128", "imm8")
    auto vaeskeygenassist(RM)(XMM dst, RM src, ubyte imm8) if (valid!(RM, 128)) => emit!(0, VEX, 128, F3A, 0x66)(0xdf, dst, src, imm8);

    /* ====== SHA ====== */

    @("r128", "rm128")
    auto sha1msg1(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xc9, dst, src);
    @("r128", "rm128")
    auto sha1msg2(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xca, dst, src);
    @("r128", "rm128")
    auto sha1nexte(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xc8, dst, src);

    @("r128", "rm128")
    auto sha256msg1(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xcc, dst, src);

    @("r128", "rm128", "imm8")
    auto sha1rnds4(RM)(XMM dst, RM src, ubyte imm8) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x3a, 0xcc, dst, src, imm8);

    @("r128", "rm128")
    auto sha256rnds2(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xcb, dst, src);

    /* ====== MAIN ====== */

    // NOTE: Branch hints are generally useless in the modern day, AMD CPUs don't even acknowledge them;
    // and thus these should not be used on any modern CPU.

    @("prefix")
    auto not_taken(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0x2e~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    @("prefix")
    auto taken(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0x3e~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    @("r32", "rm8")
    auto crc32(RM)(R32 dst, RM src) if (valid!(RM, 8)) => emit!0(0xf2, 0x0f, 0x38, 0xf0, dst, src);
    @("r32", "rm16")
    auto crc32(RM)(R32 dst, RM src) if (valid!(RM, 16)) => emit!0(0xf2, 0x0f, 0x38, 0xf1, dst, src);
    @("r32", "rm32")
    auto crc32(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf2, 0x0f, 0x38, 0xf1, dst, src);

    @("r64", "rm8")
    auto crc32(RM)(R64 dst, RM src) if (valid!(RM, 8)) => emit!0(0xf2, 0x0f, 0x38, 0xf0, dst, src);
    @("r64", "rm64")
    auto crc32(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf2, 0x0f, 0x38, 0xf1, dst, src);

    // literally 1984
    // Why did I write this comment? What is literally 1984????
    @("r32", "m512")
    auto enqcmd(R32 dst, Mem!512 src) => emit!0(0xf2, 0x0f, 0x38, 0xf8, dst, src);
    @("r64", "m512")
    auto enqcmd(R64 dst, Mem!512 src) => emit!0(0xf2, 0x0f, 0x38, 0xf8, dst, src);

    @("rm8", "r8")
    auto cmpxchg(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x0f, 0xb0, dst, src);
    @("rm16", "r16")
    auto cmpxchg(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xb1, dst, src);
    @("rm32", "r32")
    auto cmpxchg(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xb1, dst, src);
    @("rm64", "r64")
    auto cmpxchg(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xb1, dst, src);

    auto aaa() => emit!0(0x37);
    auto aad() => emit!0(0xd5, 0x0a);
    @("imm8")
    auto aad(ubyte imm8) => emit!0(0xd5, imm8);
    auto aam() => emit!0(0xd4, 0x0a);
    @("imm8")
    auto aam(ubyte imm8) => emit!0(0xd4, imm8);
    auto aas() => emit!0(0x3f);

    @("imm8")
    auto add(ubyte imm8) => emit!0(0x04, imm8);
    @("imm16")
    auto add(ushort imm16) => emit!0(0x05, imm16);
    @("imm32")
    auto add(uint imm32) => emit!0(0x05, imm32);

    @("rm8", "imm8")
    auto add(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!0(0x80, dst, imm8);
    @("rm16", "imm16")
    auto add(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!0(0x81, dst, imm16);
    @("rm32", "imm32")
    auto add(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!0(0x81, dst, imm32);
    @("rm64", "imm32")
    auto add(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!0(0x81, dst, imm32);
    @("rm16", "imm8")
    auto add(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!0(0x83, dst, imm8);
    @("rm32", "imm8")
    auto add(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!0(0x83, dst, imm8);
    @("rm64", "imm8")
    auto add(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!0(0x83, dst, imm8);

    @("rm8", "r8")
    auto add(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x00, dst, src);
    @("rm16", "r16")
    auto add(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x01, dst, src);
    @("rm32", "r32")
    auto add(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x01, dst, src);
    @("rm64", "r64")
    auto add(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x01, dst, src);

    @("r8", "m8")
    auto add(R8 dst, Mem!8 src) => emit!0(0x02, dst, src);
    @("r16", "m16")
    auto add(R16 dst, Mem!16 src) => emit!0(0x03, dst, src);
    @("r32", "m32")
    auto add(R32 dst, Mem!32 src) => emit!0(0x03, dst, src);
    @("r64", "m64")
    auto add(R64 dst, Mem!64 src) => emit!0(0x03, dst, src);

    @("imm8")
    auto and(ubyte imm8) => emit!0(0x24, imm8);
    @("imm16")
    auto and(ushort imm16) => emit!0(0x25, imm16);
    @("imm32")
    auto and(uint imm32) => emit!0(0x25, imm32);

    @("rm8", "imm8")
    auto and(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!4(0x80, dst, imm8);
    @("rm16", "imm16")
    auto and(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!4(0x81, dst, imm16);
    @("rm32", "imm32")
    auto and(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!4(0x81, dst, imm32);
    @("rm64", "imm32")
    auto and(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!4(0x81, dst, imm32);
    @("rm16", "imm8")
    auto and(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!4(0x83, dst, imm8);
    @("rm32", "imm8")
    auto and(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!4(0x83, dst, imm8);
    @("rm64", "imm8")
    auto and(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!4(0x83, dst, imm8);

    @("rm8", "r8")
    auto and(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x20, dst, src);
    @("rm16", "r16")
    auto and(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x21, dst, src);
    @("rm32", "r32")
    auto and(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x21, dst, src);
    @("rm64", "r64")
    auto and(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x21, dst, src);

    @("r8", "m8")
    auto and(R8 dst, Mem!8 src) => emit!0(0x22, dst, src)
    @("r16", "m16")
    auto and(R16 dst, Mem!16 src) => emit!0(0x23, dst, src);
    @("r32", "m32")
    auto and(R32 dst, Mem!32 src) => emit!0(0x23, dst, src);
    @("r64", "m64")
    auto and(R64 dst, Mem!64 src) => emit!0(0x23, dst, src);

    @("rm16", "r16")
    auto arpl(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x63, dst, src);

    @("r16", "rm16")
    auto bsf(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbc, dst, src);
    @("r32", "rm32")
    auto bsf(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xbc, dst, src);
    @("r64", "rm64")
    auto bsf(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0xbc, dst, src);

    @("r16", "rm16")
    auto bsr(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbd, dst, src);
    @("r32", "rm32")
    auto bsr(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xbd, dst, src);
    @("r64", "rm64")
    auto bsr(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0xbd, dst, src);

    @("r32")
    auto bswap(R32 dst) => emit!(0, NRM)(0x0f, 0xc8, dst);
    @("r64")
    auto bswap(R64 dst) => emit!(0, NRM)(0x0f, 0xc8, dst);

    @("rm16", "r16")
    auto bt(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xa3, dst, src); 
    @("rm32", "r32")
    auto bt(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xa3, dst, src); 
    @("rm64", "r64")
    auto bt(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xa3, dst, src); 
    @("rm16", "imm8")
    auto bt(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!4(0x0f, 0xba, dst, imm8); 
    @("rm32", "imm8")
    auto bt(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!4(0x0f, 0xba, dst, imm8); 
    @("rm64", "imm8")
    auto bt(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!4(0x0f, 0xba, dst, imm8); 

    @("rm16", "r16")
    auto btc(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbb, dst, src); 
    @("rm32", "r32")
    auto btc(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xbb, dst, src); 
    @("rm64", "r64")
    auto btc(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xbb, dst, src); 
    @("rm16", "imm8")
    auto btc(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!7(0x0f, 0xba, dst, imm8); 
    @("rm32", "imm8")
    auto btc(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!7(0x0f, 0xba, dst, imm8); 
    @("rm64", "imm8")
    auto btc(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!7(0x0f, 0xba, dst, imm8); 

    @("rm16", "r16")
    auto btr(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xb3, dst, src); 
    @("rm32", "r32")
    auto btr(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xb3, dst, src); 
    @("rm64", "r64")
    auto btr(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xb3, dst, src); 
    @("rm16", "imm8")
    auto btr(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!6(0x0f, 0xba, dst, imm8); 
    @("rm32", "imm8")
    auto btr(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!6(0x0f, 0xba, dst, imm8); 
    @("rm64", "imm8")
    auto btr(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!6(0x0f, 0xba, dst, imm8); 

    @("rm16", "r16")
    auto bts(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xab, dst, src); 
    @("rm32", "r32")
    auto bts(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xab, dst, src); 
    @("rm64", "r64")
    auto bts(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xab, dst, src); 
    @("rm16", "imm8")
    auto bts(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!5(0x0f, 0xba, dst, imm8); 
    @("rm32", "imm8")
    auto bts(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!5(0x0f, 0xba, dst, imm8); 
    @("rm64", "imm8")
    auto bts(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!5(0x0f, 0xba, dst, imm8);

    @("imm8")
    auto cmp(ubyte imm8) => emit!0(0x3c, imm8);
    @("imm16")
    auto cmp(ushort imm16) => emit!0(0x3d, imm16);
    @("imm32")
    auto cmp(uint imm32) => emit!0(0x3d, imm32);

    @("rm8", "imm8")
    auto cmp(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!7(0x80, dst, imm8);
    @("rm16", "imm16")
    auto cmp(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!7(0x81, dst, imm16);
    @("rm32", "imm32")
    auto cmp(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!7(0x81, dst, imm32);
    @("rm64", "imm32")
    auto cmp(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!7(0x81, dst, imm32);
    @("rm16", "imm8")
    auto cmp(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!7(0x83, dst, imm8); 
    @("rm32", "imm8")
    auto cmp(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!7(0x83, dst, imm8); 
    @("rm64", "imm8")
    auto cmp(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!7(0x83, dst, imm8); 

    @("rm8", "r8")
    auto cmp(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x38, dst, src);
    @("rm16", "r16")
    auto cmp(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x39, dst, src);
    @("rm32", "r32")
    auto cmp(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x39, dst, src);
    @("rm64", "r64")
    auto cmp(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x39, dst, src);

    @("r8", "m8")
    auto cmp(R8 dst, Mem!8 src) => emit!0(0x3a, dst, src);
    @("r16", "m16")
    auto cmp(R16 dst, Mem!16 src) => emit!0(0x3b, dst, src);
    @("r32", "m32")
    auto cmp(R32 dst, Mem!32 src) => emit!0(0x3b, dst, src);
    @("r64", "m64")
    auto cmp(R64 dst, Mem!64 src) => emit!0(0x3b, dst, src);

    auto cwd() => emit!0(0x66, 0x99);
    auto cdq() => emit!0(0x99);
    auto cqo() => emit!0(0x48, 0x99);

    auto cbw() => emit!0(0x66, 0x98);
    auto cwde() => emit!0(0x98);
    auto cdqe() => emit!0(0x48, 0x98);

    auto cpuid() => emit!0(0x0f, 0xa2);
    @("imm32")
    auto cpuid(uint imm32) => mov(eax, imm32) + cpuid();

    auto clc() => emit!0(0xf8);
    auto cld() => emit!0(0xfc);
    auto cli() => emit!0(0xfa);
    auto clts() => emit!0(0x0f, 0x06);

    auto cmc() => emit!0(0xf5);

    @("rm8")
    auto dec(RM)(RM dst) if (valid!(RM, 8)) => emit!1(0xfe, dst);
    static if (X64)
    @("rm16")
    auto dec(RM)(RM dst) if (valid!(RM, 16)) => emit!1(0xff, dst);
    static if (!X64)
    @("m16")
    auto dec(Mem!16 dst) => emit!1(0xff, dst);
    static if (X64)
    @("rm32")
    auto dec(RM)(RM dst) if (valid!(RM, 32)) => emit!1(0xff, dst);
    static if (!X64)
    @("m32")
    auto dec(Mem!32 dst) => emit!1(0xff, dst);
    @("rm64")
    auto dec(RM)(RM dst) if (valid!(RM, 64)) => emit!1(0xff, dst);

    static if (!X64)
    @("r16")
    auto dec(R16 dst) => emit!(0, NRM)(0x48, dst);
    static if (!X64)
    @("r32")
    auto dec(R32 dst) => emit!(0, NRM)(0x48, dst);

    auto int3() => emit!0(0xcc);
    @("imm8")
    auto _int(ubyte imm8) => emit!0(0xcd, imm8);
    auto into() => emit!0(0xce);
    auto int1() => emit!0(0xf1);
    @("r32", "rm32")
    auto ud0(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xff, dst, src);
    @("r32", "rm32")
    auto ud1(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xb9, dst, src);
    auto ud2() => emit!0(0x0f, 0x0b);
    
    auto iret() => emit!0(0xcf);
    auto iretd() => emit!0(0xcf);
    auto iretq() => emit!0(0xcf);

    @("rm8")
    auto inc(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0xfe, dst);
    static if (X64)
    @("rm16")
    auto inc(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0xff, dst);
    static if (!X64)
    @("m16")
    auto inc(Mem!16 dst) => emit!0(0xff, dst);
    static if (X64)
    @("rm32")
    auto inc(RM)(RM dst) if (valid!(RM, 32)) => emit!0(0xff, dst);
    static if (!X64)
    @("m32")
    auto inc(Mem!32 dst) => emit!0(0xff, dst);
    @("rm64")
    auto inc(RM)(RM dst) if (valid!(RM, 64)) => emit!0(0xff, dst);

    static if (!X64)
    @("r16")
    auto inc(R16 dst) => emit!(0, NRM)(0x40, dst);
    static if (!X64)
    @("r32")
    auto inc(R32 dst) => emit!(0, NRM)(0x40, dst);

    auto hlt() => emit!0(0xf4);
    auto pause() => emit!0(0xf3, 0x90);
    auto swapgs() => emit!0(0x0f, 0x01, 0xf8);
    
    @("prefix")
    auto lock(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf0~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    auto wait() => emit!0(0x9b);
    auto fwait() => emit!0(0x9b);

    auto sysretc() => emit!0(0x0f, 0x07);
    auto sysret() => emit!0(0x0f, 0x07);
    auto syscall() => emit!0(0x0f, 0x05);
    auto rsm() => emit!0(0x0f, 0xaa);

    auto leave() => emit!0(0xc9);
    @("imm16")
    auto enter(ushort imm16) => emit!0(0xc8, imm16, 0x00);
    @("imm16", "imm8")
    auto enter(ushort imm16, ubyte imm8) => emit!0(0xc8, imm16, imm8);
    
    @("r16", "m16")
    auto lea(RM)(R16 dst, Mem!16) => emit!0(0x8d, dst, src);
    @("r32", "m32")
    auto lea(RM)(R32 dst, Mem!32) => emit!0(0x8d, dst, src);
    @("r64", "m64")
    auto lea(RM)(R64 dst, Mem!64) => emit!0(0x8d, dst, src);

    @("r16", "m16")
    auto lds(RM)(R16 dst, Mem!16) => emit!0(0xc5, dst, src);
    @("r32", "m32")
    auto lds(RM)(R32 dst, Mem!32) => emit!0(0xc5, dst, src);

    @("r16", "m16")
    auto lss(RM)(R16 dst, Mem!16) => emit!0(0x0f, 0xb2, dst, src);
    @("r32", "m32")
    auto lss(RM)(R32 dst, Mem!32) => emit!0(0x0f, 0xb2, dst, src);
    @("r64", "m64")
    auto lss(RM)(R64 dst, Mem!64) => emit!0(0x0f, 0xb2, dst, src);

    @("r16", "m16")
    auto les(RM)(R16 dst, Mem!16) => emit!0(0xc4, dst, src);
    @("r32", "m32")
    auto les(RM)(R32 dst, Mem!32) => emit!0(0xc4, dst, src);

    @("r16", "m16")
    auto lfs(RM)(R16 dst, Mem!16) => emit!0(0x0f, 0xb4, dst, src);
    @("r32", "m32")
    auto lfs(RM)(R32 dst, Mem!32) => emit!0(0x0f, 0xb4, dst, src);
    @("r64", "m64")
    auto lfs(RM)(R64 dst, Mem!64) => emit!0(0x0f, 0xb4, dst, src);

    @("r16", "m16")
    auto lgs(RM)(R16 dst, Mem!16) => emit!0(0x0f, 0xb5, dst, src);
    @("r32", "m32")
    auto lgs(RM)(R32 dst, Mem!32) => emit!0(0x0f, 0xb5, dst, src);
    @("r64", "m64")
    auto lgs(RM)(R64 dst, Mem!64) => emit!0(0x0f, 0xb5, dst, src);

    @("r16", "rm16")
    auto lsl(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x03, dst, src);
    @("r32", "r32")
    auto lsl(R32 dst, R32 src) => emit!0(0x0f, 0x03, dst, src);
    @("r64", "r32")
    auto lsl(R64 dst, R32 src) => emit!0(0x0f, 0x03, dst, src);
    @("r32", "m16")
    auto lsl(R32 dst, Mem!16 src) => emit!0(0x0f, 0x03, dst, src);
    @("r64", "m16")
    auto lsl(R64 dst, Mem!16 src) => emit!0(0x0f, 0x03, dst, src);

    @("rm16")
    auto ltr(RM)(RM dst) if (valid!(RM, 16)) => emit!3(0x0f, 0x00, dst);
    @("rm16")
    auto str(RM)(RM dst) if (valid!(RM, 16)) => emit!1(0x0f, 0x00, dst);

    @("rm8")
    auto neg(RM)(RM dst) if (valid!(RM, 8)) => emit!3(0xf6, dst);
    @("rm16")
    auto neg(RM)(RM dst) if (valid!(RM, 16)) => emit!3(0xf7, dst);
    @("rm32")
    auto neg(RM)(RM dst) if (valid!(RM, 32)) => emit!3(0xf7, dst);
    @("rm64")
    auto neg(RM)(RM dst) if (valid!(RM, 64)) => emit!3(0xf7, dst);

    auto nop() => emit!0(0x90);
    @("rm16")
    auto nop(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0x0f, 0x1f, dst);

    @("rm8")
    auto not(RM)(RM dst) if (valid!(RM, 8)) => emit!2(0xf6, dst);
    @("rm16")
    auto not(RM)(RM dst) if (valid!(RM, 16)) => emit!2(0xf7, dst);
    @("rm32")
    auto not(RM)(RM dst) if (valid!(RM, 32)) => emit!2(0xf7, dst);
    @("rm64")
    auto not(RM)(RM dst) if (valid!(RM, 64)) => emit!2(0xf7, dst);

    auto ret() => emit!0(0xc3);
    @("imm16")
    auto ret(ushort imm16) => emit!0(0xc2, imm16);
    auto retf() => emit!0(0xcb);
    @("imm16")
    auto retf(ushort imm16) => emit!0(0xca, imm16);

    auto stc() => emit!0(0xf9);
    auto std() => emit!0(0xfd);
    auto sti() => emit!0(0xfb);

    @("imm8")
    auto sub(ubyte imm8) => emit!0(0x2c, imm8);
    @("imm16")
    auto sub(ushort imm16) => emit!0(0x2d, imm16);
    @("imm32")
    auto sub(uint imm32) => emit!0(0x2d, imm32);

    @("rm8", "imm8")
    auto sub(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!5(0x80, dst, imm8);
    @("rm16", "imm16")
    auto sub(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!5(0x81, dst, imm16);
    @("rm32", "imm32")
    auto sub(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!5(0x81, dst, imm32);
    @("rm64", "imm32")
    auto sub(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!5(0x81, dst, imm32);
    @("rm16", "imm8")
    auto sub(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!5(0x83, dst, imm8);
    @("rm32", "imm8")
    auto sub(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!5(0x83, dst, imm8);
    @("rm64", "imm8")
    auto sub(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!5(0x83, dst, imm8);

    @("rm8", "r8")
    auto sub(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x28, dst, src);
    @("rm16", "r16")
    auto sub(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x29, dst, src);
    @("rm32", "r32")
    auto sub(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x29, dst, src);
    @("rm64", "r64")
    auto sub(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x29, dst, src);

    @("r8", "m8")
    auto sub(R8 dst, Mem!8 src) => emit!0(0x2a, dst, src);
    @("r16", "m16")
    auto sub(R16 dst, Mem!16 src) => emit!0(0x2b, dst, src);
    @("r32", "m32")
    auto sub(R32 dst, Mem!32 src) => emit!0(0x2b, dst, src);
    @("r64", "m64")
    auto sub(R64 dst, Mem!64 src) => emit!0(0x2b, dst, src);

    @("imm8")
    auto sbb(ubyte imm8) => emit!0(0x1c, imm8);
    @("imm16")
    auto sbb(ushort imm16) => emit!0(0x1d, imm16);
    @("imm32")
    auto sbb(uint imm32) => emit!0(0x1d, imm32);

    @("rm8", "imm8")
    auto sbb(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!3(0x80, dst, imm8);
    @("rm16", "imm16")
    auto sbb(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!3(0x81, dst, imm16);
    @("rm32", "imm32")
    auto sbb(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!3(0x81, dst, imm32);
    @("rm64", "imm32")
    auto sbb(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!3(0x81, dst, imm32);
    @("rm16", "imm8")
    auto sbb(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!3(0x83, dst, imm8);
    @("rm32", "imm8")
    auto sbb(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!3(0x83, dst, imm8);
    @("rm64", "imm8")
    auto sbb(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!3(0x83, dst, imm8);

    @("rm8", "r8")
    auto sbb(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x18, dst, src);
    @("rm16", "r16")
    auto sbb(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x19, dst, src);
    @("rm32", "r32")
    auto sbb(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x19, dst, src);
    @("rm64", "r64")
    auto sbb(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x19, dst, src);

    @("r8", "m8")
    auto sbb(R8 dst, Mem!8 src) => emit!0(0x1a, dst, src);
    @("r16", "m16")
    auto sbb(R16 dst, Mem!16 src) => emit!0(0x1b, dst, src);
    @("r32", "m32")
    auto sbb(R32 dst, Mem!32 src) => emit!0(0x1b, dst, src);
    @("r64", "m64")
    auto sbb(R64 dst, Mem!64 src) => emit!0(0x1b, dst, src);

    @("imm8")
    auto xor(ubyte imm8) => emit!0(0x34, imm8);
    @("imm16")
    auto xor(ushort imm16) => emit!0(0x35, imm16);
    @("imm32")
    auto xor(uint imm32) => emit!0(0x35, imm32);

    @("r8", "imm8")
    auto xor(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!6(0x80, dst, imm8);
    @("r16", "imm16")
    auto xor(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!6(0x81, dst, imm16);
    @("r32", "imm32")
    auto xor(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!6(0x81, dst, imm32);
    @("r64", "imm32")
    auto xor(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!6(0x81, dst, imm32);
    @("r16", "imm8")
    auto xor(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!6(0x83, dst, imm8);
    @("r32", "imm8")
    auto xor(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!6(0x83, dst, imm8);
    @("r64", "imm8")
    auto xor(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!6(0x83, dst, imm8);

    @("rm8", "r8")
    auto xor(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x30, dst, src);
    @("rm16", "r16")
    auto xor(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x31, dst, src);
    @("rm32", "r32")
    auto xor(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x31, dst, src);
    @("rm64", "r64")
    auto xor(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x31, dst, src);

    @("r8", "m8")
    auto xor(R8 dst, Mem!8 src) => emit!0(0x32, dst, src);
    @("r16", "m16")
    auto xor(R16 dst, Mem!16 src) => emit!0(0x33, dst, src);
    @("r32", "m32")
    auto xor(R32 dst, Mem!32 src) => emit!0(0x33, dst, src);
    @("r64", "m64")
    auto xor(R64 dst, Mem!64 src) => emit!0(0x33, dst, src);

    @("imm8")
    auto or(ubyte imm8) => emit!0(0x0c, imm8);
    @("imm16")
    auto or(ushort imm16) => emit!0(0x0d, imm16);
    @("imm32")
    auto or(uint imm32) => emit!0(0x0d, imm32);

    @("r8", "imm8")
    auto or(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!1(0x80, dst, imm8);
    @("r16", "imm16")
    auto or(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!1(0x81, dst, imm16);
    @("r32", "imm32")
    auto or(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!1(0x81, dst, imm32);
    @("r64", "imm32")
    auto or(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!1(0x81, dst, imm32);
    @("r16", "imm8")
    auto or(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!1(0x83, dst, imm8);
    @("r32", "imm8")
    auto or(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!1(0x83, dst, imm8);
    @("r64", "imm8")
    auto or(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!1(0x83, dst, imm8);

    @("rm8", "r8")
    auto or(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x8, dst, src);
    @("rm16", "r16")
    auto or(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x9, dst, src);
    @("rm32", "r32")
    auto or(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x9, dst, src);
    @("rm64", "r64")
    auto or(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x9, dst, src);

    @("r8", "m8")
    auto or(R8 dst, Mem!8 src) => emit!0(0xa, dst, src);
    @("r16", "m16")
    auto or(R16 dst, Mem!16 src) => emit!0(0xb, dst, src);
    @("r32", "m32")
    auto or(R32 dst, Mem!32 src) => emit!0(0xb, dst, src);
    @("r64", "m64")
    auto or(R64 dst, Mem!64 src) => emit!0(0xb, dst, src);

    @("rm8")
    auto sal(RM)(RM dst) if (valid!(RM, 8)) => emit!4(0xd2, dst, cl);
    @("rm8", "imm8")
    auto sal(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!4(0xd0, dst);
        else
            return emit!4(0xc0, dst, imm8);
    }
    @("rm16")
    auto sal(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0xd3, dst, cl);
    @("rm16", "imm8")
    auto sal(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!4(0xd1, dst);
        else
            return emit!4(0xc1, dst, imm8);
    }
    @("rm32")
    auto sal(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0xd3, dst, cl);
    @("rm32", "imm8")
    auto sal(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!4(0xd1, dst);
        else
            return emit!4(0xc1, dst, imm8);
    }
    @("rm64")
    auto sal(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0xd3, dst, cl);
    @("rm64", "imm8")
    auto sal(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!4(0xd1, dst);
        else
            return emit!4(0xc1, dst, imm8);
    }

    @("rm8")
    auto sar(RM)(RM dst) if (valid!(RM, 8)) => emit!7(0xd2, dst, cl);
    @("rm8", "imm8")
    auto sar(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!7(0xd0, dst);
        else
            return emit!7(0xc0, dst, imm8);
    }
    @("rm16")
    auto sar(RM)(RM dst) if (valid!(RM, 16)) => emit!7(0xd3, dst, cl);
    @("rm16", "imm8")
    auto sar(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!7(0xd1, dst);
        else
            return emit!7(0xc1, dst, imm8);
    }
    @("rm32")
    auto sar(RM)(RM dst) if (valid!(RM, 32)) => emit!7(0xd3, dst, cl);
    @("rm32", "imm8")
    auto sar(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!7(0xd1, dst);
        else
            return emit!7(0xc1, dst, imm8);
    }
    @("rm64")
    auto sar(RM)(RM dst) if (valid!(RM, 64)) => emit!7(0xd3, dst, cl);
    @("rm64", "imm8")
    auto sar(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!7(0xd1, dst);
        else
            return emit!7(0xc1, dst, imm8);
    }

    @("rm8")
    auto shl(RM)(RM dst) if (valid!(RM, 8)) => sal(dst);
    @("rm8", "imm8")
    auto shl(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => sal(dst, imm8);
    @("rm16")
    auto shl(RM)(RM dst) if (valid!(RM, 16)) => sal(dst);
    @("rm16", "imm8")
    auto shl(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => sal(dst, imm8);
    @("rm32")
    auto shl(RM)(RM dst) if (valid!(RM, 32)) => sal(dst);
    @("rm32", "imm8")
    auto shl(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => sal(dst, imm8);
    @("rm64")
    auto shl(RM)(RM dst) if (valid!(RM, 64)) => sal(dst);
    @("rm64", "imm8")
    auto shl(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => sal(dst, imm8);

    @("rm8")
    auto shr(RM)(RM dst) if (valid!(RM, 8)) => emit!5(0xd2, dst, cl);
    @("rm8", "imm8")
    auto shr(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!5(0xd0, dst);
        else
            return emit!5(0xc0, dst, imm8);
    }
    @("rm16")
    auto shr(RM)(RM dst) if (valid!(RM, 16)) => emit!5(0xd3, dst, cl);
    @("rm16", "imm8")
    auto shr(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!5(0xd1, dst);
        else
            return emit!5(0xc1, dst, imm8);
    }
    @("rm32")
    auto shr(RM)(RM dst) if (valid!(RM, 32)) => emit!5(0xd3, dst, cl);
    @("rm32", "imm8")
    auto shr(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!5(0xd1, dst);
        else
            return emit!5(0xc1, dst, imm8);
    }
    @("rm64")
    auto shr(RM)(RM dst) if (valid!(RM, 64)) => emit!5(0xd3, dst, cl);
    @("rm64", "imm8")
    auto shr(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!5(0xd1, dst);
        else
            return emit!5(0xc1, dst, imm8);
    }

    @("rm8")
    auto rcl(RM)(RM dst) if (valid!(RM, 8)) => emit!2(0xd2, dst, cl);
    @("rm8", "imm8")
    auto rcl(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!2(0xd0, dst);
        else
            return emit!2(0xc0, dst, imm8);
    }
    @("rm16")
    auto rcl(RM)(RM dst) if (valid!(RM, 16)) => emit!2(0xd3, dst, cl);
    @("rm16", "imm8")
    auto rcl(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!2(0xd1, dst);
        else
            return emit!2(0xc1, dst, imm8);
    }
    @("rm32")
    auto rcl(RM)(RM dst) if (valid!(RM, 32)) => emit!2(0xd3, dst, cl);
    @("rm32", "imm8")
    auto rcl(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!2(0xd1, dst);
        else
            return emit!2(0xc1, dst, imm8);
    }
    @("rm64")
    auto rcl(RM)(RM dst) if (valid!(RM, 64)) => emit!2(0xd3, dst, cl);
    @("rm64", "imm8")
    auto rcl(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!2(0xd1, dst);
        else
            return emit!2(0xc1, dst, imm8);
    }

    @("rm8")
    auto rcr(RM)(RM dst) if (valid!(RM, 8)) => emit!3(0xd2, dst, cl);
    @("rm8", "imm8")
    auto rcr(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!3(0xd0, dst);
        else
            return emit!3(0xc0, dst, imm8);
    }
    @("rm16")
    auto rcr(RM)(RM dst) if (valid!(RM, 16)) => emit!3(0xd3, dst, cl);
    @("rm16", "imm8")
    auto rcr(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!3(0xd1, dst);
        else
            return emit!3(0xc1, dst, imm8);
    }
    @("rm32")
    auto rcr(RM)(RM dst) if (valid!(RM, 32)) => emit!3(0xd3, dst, cl);
    @("rm32", "imm8")
    auto rcr(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!3(0xd1, dst);
        else
            return emit!3(0xc1, dst, imm8);
    }
    @("rm64")
    auto rcr(RM)(RM dst) if (valid!(RM, 64)) => emit!3(0xd3, dst, cl);
    @("rm64", "imm8")
    auto rcr(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!3(0xd1, dst);
        else
            return emit!3(0xc1, dst, imm8);
    }

    @("rm8")
    auto rol(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0xd2, dst, cl);
    @("rm8", "imm8")
    auto rol(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!0(0xd0, dst);
        else
            return emit!0(0xc0, dst, imm8);
    }
    @("rm16")
    auto rol(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0xd3, dst, cl);
    @("rm16", "imm8")
    auto rol(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!0(0xd1, dst);
        else
            return emit!0(0xc1, dst, imm8);
    }
    @("rm32")
    auto rol(RM)(RM dst) if (valid!(RM, 32)) => emit!0(0xd3, dst, cl);
    @("rm32", "imm8")
    auto rol(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!0(0xd1, dst);
        else
            return emit!0(0xc1, dst, imm8);
    }
    @("rm64")
    auto rol(RM)(RM dst) if (valid!(RM, 64)) => emit!0(0xd3, dst, cl);
    @("rm64", "imm8")
    auto rol(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!0(0xd1, dst);
        else
            return emit!0(0xc1, dst, imm8);
    }

    @("rm8")
    auto ror(RM)(RM dst) if (valid!(RM, 8)) => emit!1(0xd2, dst, cl);
    @("rm8", "imm8")
    auto ror(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!1(0xd0, dst);
        else
            return emit!1(0xc0, dst, imm8);
    }
    @("rm16")
    auto ror(RM)(RM dst) if (valid!(RM, 16)) => emit!1(0xd3, dst, cl);
    @("rm16", "imm8")
    auto ror(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!1(0xd1, dst);
        else
            return emit!1(0xc1, dst, imm8);
    }
    @("rm32")
    auto ror(RM)(RM dst) if (valid!(RM, 32)) => emit!1(0xd3, dst, cl);
    @("rm32", "imm8")
    auto ror(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!1(0xd1, dst);
        else
            return emit!1(0xc1, dst, imm8);
    }
    @("rm64")
    auto ror(RM)(RM dst) if (valid!(RM, 64)) => emit!1(0xd3, dst, cl);
    @("rm64", "imm8")
    auto ror(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!1(0xd1, dst);
        else
            return emit!1(0xc1, dst, imm8);
    }

    @("rm16")
    auto verr(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0xf0, 0x00, dst);
    @("rm16")
    auto verw(RM)(RM dst) if (valid!(RM, 16)) => emit!5(0xf0, 0x00, dst);

    @("imm8")
    auto test(ubyte imm8) => emit!0(0xa8, imm8);
    @("imm16")
    auto test(ushort imm16) => emit!0(0xa9, imm16);
    @("imm32")
    auto test(uint imm32) => emit!0(0xa9, imm32);

    @("rm8", "imm8")
    auto test(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!0(0xf6, dst, imm8);
    @("rm16", "imm16")
    auto test(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!0(0xf7, dst, imm16);
    @("rm32", "imm32")
    auto test(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!0(0xf7, dst, imm32);
    @("rm64", "imm32")
    auto test(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!0(0xf7, dst, cast(long)imm32);

    @("rm8", "r8")
    auto test(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x84, dst, src);
    @("rm16", "r16")
    auto test(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x85, dst, src);
    @("rm32", "r32")
    auto test(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x85, dst, src);
    @("rm64", "r64")
    auto test(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x85, dst, src);

    @("m16")
    auto pop(Mem!16 dst) => emit!0(0x8f, dst);
    static if (!X64)
    @("m32")
    auto pop(Mem!32 dst) => emit!(0, NP)(0x8f, dst);
    static if (X64)
    @("m64")
    auto pop(Mem!64 dst) => emit!(0, NP)(0x8f, dst);

    @("r16")
    auto pop(R16 dst) => emit!(0, NRM)(0x58, dst);
    static if (!X64)
    @("r32")
    auto pop(R32 dst) => emit!(0, NRM)(0x58, dst);
    static if (X64)
    @("r64")
    auto pop(R64 dst) => emit!(0, NRM)(0x58, dst);

    auto popds() => emit!0(0x1f);
    auto popes() => emit!0(0x07);
    auto popss() => emit!0(0x17);
    auto popfs() => emit!0(0x0f, 0xa1);
    auto popgs() => emit!0(0x0f, 0xa9); 

    auto popa() => emit!0(0x61);
    auto popad() => emit!0(0x61);

    auto popf() => emit!0(0x9d);
    auto popfd() => emit!0(0x9d);
    auto popfq() => emit!0(0x9d);

    @("m16")
    auto push(Mem!16 dst) => emit!6(0xff, dst);
    static if (!X64)
    @("m32")
    auto push(Mem!32 dst) => emit!(6, NP)(0xff, dst);
    static if (X64)
    @("m64")
    auto push(Mem!64 dst) => emit!(6, NP)(0xff, dst);

    @("r16")
    auto push(R16 dst) => emit!(0, NP)(0x50, dst);
    static if (!X64)
    @("r32")
    auto push(R32 dst) => emit!(0, NP)(0x50, dst);
    static if (X64)
    @("r64")
    auto push(R64 dst) => emit!(0, NP)(0x50, dst);

    @("imm8")
    auto push(ubyte imm8) => emit!0(0x6a, imm8);
    @("imm16")
    auto push(ushort imm16) => emit!0(0x68, imm16);
    @("imm32")
    auto push(uint imm32) => emit!0(0x68, imm32);

    auto pushcs() => emit!0(0x0e);
    auto pushss() => emit!0(0x16);
    auto pushds() => emit!0(0x1e);
    auto pushes() => emit!0(0x06);
    auto pushfs() => emit!0(0x0f, 0xa0);
    auto pushgs() => emit!0(0x0f, 0xa8); 

    auto pusha() => emit!0(0x60);
    auto pushad() => emit!0(0x60);

    auto pushf() => emit!0(0x9c);
    auto pushfd() => emit!0(0x9c);
    auto pushfq() => emit!0(0x9c);

    @("rm8", "r8")
    auto xadd(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x0f, 0xc0, dst, src);
    @("rm16", "r16")
    auto xadd(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xc1, dst, src);
    @("rm32", "r32")
    auto xadd(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xc1, dst, src);
    @("rm64", "r64")
    auto xadd(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xc1, dst, src);

    @("r16")
    auto xchg(R16 dst) => emit!(0, NRM)(90, dst);
    @("r32")
    auto xchg(R32 dst) => emit!(0, NRM)(90, dst);
    @("r64")
    auto xchg(R64 dst) => emit!(0, NRM)(90, dst);

    @("rm8", "rm8")
    auto xchg(A, B)(A dst, B src) if (valid!(A, 8) && valid!(B, 8)) => emit!0(0x86, dst, src);
    @("rm16", "rm16")
    auto xchg(A, B)(A dst, B src) if (valid!(A, 16) && valid!(B, 16)) => emit!0(0x87, dst, src);
    @("rm32", "rm32")
    auto xchg(A, B)(A dst, B src) if (valid!(A, 32) && valid!(B, 32)) => emit!0(0x87, dst, src);
    @("rm64", "rm64")
    auto xchg(A, B)(A dst, B src) if (valid!(A, 64) && valid!(B, 64)) => emit!0(0x87, dst, src);

    auto xlat() => emit!0(0xd7);
    static if (!X64)
    auto xlatb() => emit!0(0xd7);
    static if (X64)
    auto xlatb() => emit!0(0x48, 0xd7);

    @("r16", "m16")
    auto lar(R16 dst, Mem!16 src) => emit!0(0x0f, 0x02, dst, src);
    @("r16", "r16")
    auto lar(R16 dst, R16 src) => emit!0(0x0f, 0x02, dst, src);
    @("r32", "m16")
    auto lar(R32 dst, Mem!16 src) => emit!0(0x0f, 0x02, dst, src);
    @("r32", "r32")
    auto lar(R32 dst, R32 src) => emit!0(0x0f, 0x02, dst, src);

    auto daa() => emit!0(0x27);
    auto das() => emit!0(0x2f);

    @("rm8")
    auto mul(RM)(RM dst) if (valid!(RM, 8)) => emit!4(0xf6, dst);
    @("rm16")
    auto mul(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0xf7, dst);
    @("rm32")
    auto mul(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0xf7, dst);
    @("rm64")
    auto mul(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0xf7, dst);

    @("rm8")
    auto imul(RM)(RM dst) if (valid!(RM, 8)) => emit!5(0xf6, dst);
    @("rm16")
    auto imul(RM)(RM dst) if (valid!(RM, 16)) => emit!5(0xf7, dst);
    @("rm32")
    auto imul(RM)(RM dst) if (valid!(RM, 32)) => emit!5(0xf7, dst);
    @("rm64")
    auto imul(RM)(RM dst) if (valid!(RM, 64)) => emit!5(0xf7, dst);

    @("r16", "rm16")
    auto imul(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xaf, dst, src);
    @("r32", "rm32")
    auto imul(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xaf, dst, src);
    @("r64", "rm64")
    auto imul(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0xaf, dst, src);

    @("r16", "rm16", "imm8")
    auto imul(RM)(R16 dst, RM src, ubyte imm8) if (valid!(RM, 16)) => emit!0(0x6b, dst, src, imm8);
    @("r32", "rm32", "imm8")
    auto imul(RM)(R32 dst, RM src, ubyte imm8) if (valid!(RM, 32)) => emit!0(0x6b, dst, src, imm8);
    @("r64", "rm64", "imm8")
    auto imul(RM)(R64 dst, RM src, ubyte imm8) if (valid!(RM, 64)) => emit!0(0x6b, dst, src, imm8);
    @("r16", "rm16", "imm16")
    auto imul(RM)(R16 dst, RM src, ushort imm16) if (valid!(RM, 16)) => emit!0(0x69, dst, src, imm16);
    @("r32", "rm32", "imm32")
    auto imul(RM)(R32 dst, RM src, uint imm32) if (valid!(RM, 32)) => emit!0(0x69, dst, src, imm32);
    @("r64", "rm64", "imm32")
    auto imul(RM)(R64 dst, RM src, uint imm32) if (valid!(RM, 64)) => emit!0(0x69, dst, src, imm32);

    @("rm8")
    auto div(RM)(RM dst) if (valid!(RM, 8)) => emit!6(0xf6, dst);
    @("rm16")
    auto div(RM)(RM dst) if (valid!(RM, 16)) => emit!6(0xf7, dst);
    @("rm32")
    auto div(RM)(RM dst) if (valid!(RM, 32)) => emit!6(0xf7, dst);
    @("rm64")
    auto div(RM)(RM dst) if (valid!(RM, 64)) => emit!6(0xf7, dst);

    @("rm8")
    auto idiv(RM)(RM dst) if (valid!(RM, 8)) => emit!7(0xf6, dst);
    @("rm16")
    auto idiv(RM)(RM dst) if (valid!(RM, 16)) => emit!7(0xf7, dst);
    @("rm32")
    auto idiv(RM)(RM dst) if (valid!(RM, 32)) => emit!7(0xf7, dst);
    @("rm64")
    auto idiv(RM)(RM dst) if (valid!(RM, 64)) => emit!7(0xf7, dst);

    @("rm8", "r8")
    auto mov(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x88, dst, src);
    @("rm16", "r16")
    auto mov(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x89, dst, src);
    @("rm32", "r32")
    auto mov(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x89, dst, src);
    @("rm64", "r64")
    auto mov(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x89, dst, src);

    @("r8", "m8")
    auto mov(R8 dst, Mem!8 src) => emit!0(0x8a, dst, src);
    @("r16", "m16")
    auto mov(R16 dst, Mem!16 src) => emit!0(0x8b, dst, src);
    @("r32", "m32")
    auto mov(R32 dst, Mem!32 src) => emit!0(0x8b, dst, src);
    @("r64", "m64")
    auto mov(R64 dst, Mem!64 src) => emit!0(0x8b, dst, src);
    
    @("r8", "imm8")
    auto mov(R8 dst, ubyte imm8) => emit!(0, NRM)(0xb0, dst, imm8);
    @("r16", "imm16")
    auto mov(R16 dst, ushort imm16) => emit!(0, NRM)(0xb8, dst, imm16);
    @("r32", "imm32")
    auto mov(R32 dst, uint imm32) => emit!(0, NRM)(0xb8, dst, imm32);
    @("r64", "imm64")
    auto mov(R64 dst, ulong imm64) => emit!(0, NRM)(0xb8, dst, imm64);

    @("m8", "imm8")
    auto mov(Mem!8 dst, ubyte imm8) => emit!0(0xc6, dst, imm8);
    @("m16", "imm16")
    auto mov(Mem!16 dst, ushort imm16) => emit!0(0xc7, dst, imm16);
    @("m32", "imm32")
    auto mov(Mem!32 dst, uint imm32) => emit!0(0xc7, dst, imm32);
    @("m64", "imm32")
    auto mov(Mem!64 dst, uint imm32) => emit!0(0xc7, dst, imm32);

    @("r32", "cr")
    auto mov(R32 dst, CR src) => emit!0(0x0f, 0x20, dst, src);
    @("r64", "cr")
    auto mov(R64 dst, CR src) => emit!0(0x0f, 0x20, dst, src);
    @("cr", "r32")
    auto mov(CR dst, R32 src) => emit!0(0x0f, 0x22, dst, src);
    @("cr", "r64")
    auto mov(CR dst, R64 src) => emit!0(0x0f, 0x22, dst, src);

    @("r32", "dr")
    auto mov(R32 dst, DR src) => emit!0(0x0f, 0x21, dst, src);
    @("r64", "dr")
    auto mov(R64 dst, DR src) => emit!0(0x0f, 0x21, dst, src);
    @("dr", "r32")
    auto mov(DR dst, R32 src) => emit!0(0x0f, 0x23, dst, src);
    @("dr", "r64")
    auto mov(DR dst, R64 src) => emit!0(0x0f, 0x23, dst, src);

    @("r16", "rm8")
    auto movsx(RM)(R16 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xbe, dst, src);
    @("r32", "rm8")
    auto movsx(RM)(R32 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xbe, dst, src);
    @("r64", "rm8")
    auto movsx(RM)(R64 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xbe, dst, src);

    @("r32", "rm16")
    auto movsx(RM)(R32 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbf, dst, src);
    @("r64", "rm16")
    auto movsx(RM)(R64 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbf, dst, src);

    @("r16", "rm16")
    auto movsxd(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x63, dst, src);
    @("r32", "rm32")
    auto movsxd(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x63, dst, src);
    @("r64", "rm32")
    auto movsxd(RM)(R64 dst, RM src) if (valid!(RM, 32)) => emit!0(0x63, dst, src);

    @("r16", "rm8")
    auto movzx(RM)(R16 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xb6, dst, src);
    @("r32", "rm8")
    auto movzx(RM)(R32 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xb6, dst, src);
    @("r64", "rm8")
    auto movzx(RM)(R64 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xb6, dst, src);

    @("r32", "rm16")
    auto movzx(RM)(R32 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xb7, dst, src);
    @("r64", "rm16")
    auto movzx(RM)(R64 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xb7, dst, src);

    @("imm16")
    auto call(ushort rel16) => emit!0(0xe8, rel16);
    @("imm32")
    auto call(uint rel32) => emit!0(0xe8, rel32);

    @("r16")
    auto call(R16 dst) => emit!2(0xff, dst);
    @("r32")
    auto call(R32 dst) => emit!2(0xff, dst);
    @("r64")
    auto call(R64 dst) => emit!2(0xff, dst);

    @("m16")
    auto call(Mem!16 dst) => emit!3(0xff, dst);
    @("m32")
    auto call(Mem!32 dst) => emit!3(0xff, dst);
    @("m64")
    auto call(Mem!64 dst) => emit!3(0xff, dst);

    @("jump")
    auto loop(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loop", name !in labels);
    @("jump")
    auto loope(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loope", name !in labels);
    @("jump")
    auto loopne(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loopne", name !in labels);

    @("jump")
    auto jmp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jmp", name !in labels);
    @("rm16")
    auto jmp(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0xff, dst);
    @("rm32")
    auto jmp(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0xff, dst);
    @("rm64")
    auto jmp(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0xff, dst);

    /* auto jmp(Mem!16 dst) => emit!5(0xff, dst);
    auto jmp(Mem!32 dst) => emit!5(0xff, dst);
    auto jmp(Mem!64 dst) => emit!5(0xff, dst); */

    @("imm16")
    auto jmp(ushort imm16) => emit!0(0xea, imm16);
    @("imm32")
    auto jmp(uint imm32) => emit!0(0xea, imm32);

    @("jump")
    auto ja(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "ja", name !in labels);
    @("jump")
    auto jae(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jae", name !in labels);
    @("jump")
    auto jb(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jb", name !in labels);
    @("jump")
    auto jbe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jbe", name !in labels);
    @("jump")
    auto jc(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jc", name !in labels);
    @("jump")
    auto jcxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jcxz", name !in labels);
    @("jump")
    auto jecxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jecxz", name !in labels);
    @("jump")
    auto jrcxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jrcxz", name !in labels);
    @("jump")
    auto je(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "je", name !in labels);
    @("jump")
    auto jg(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jg", name !in labels);
    @("jump")
    auto jge(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jge", name !in labels);
    @("jump")
    auto jl(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jl", name !in labels);
    @("jump")
    auto jle(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jle", name !in labels);
    @("jump")
    auto jna(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jna", name !in labels);
    @("jump")
    auto jnae(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnae", name !in labels);
    @("jump")
    auto jnb(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnb", name !in labels);
    @("jump")
    auto jnbe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnbe", name !in labels);
    @("jump")
    auto jnc(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnc", name !in labels);
    @("jump")
    auto jne(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jne", name !in labels);
    @("jump")
    auto jng(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jng", name !in labels);
    @("jump")
    auto jnge(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnge", name !in labels);
    @("jump")
    auto jnl(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnl", name !in labels);
    @("jump")
    auto jnle(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnle", name !in labels);
    @("jump")
    auto jno(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jno", name !in labels);
    @("jump")
    auto jnp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnp", name !in labels);
    @("jump")
    auto jns(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jns", name !in labels);
    @("jump")
    auto jnz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnz", name !in labels);
    @("jump")
    auto jo(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jo", name !in labels);
    @("jump")
    auto jp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jp", name !in labels);
    @("jump")
    auto jpe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jpe", name !in labels);
    @("jump")
    auto jpo(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jpo", name !in labels);
    @("jump")
    auto js(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "js", name !in labels);
    @("jump")
    auto jz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jz", name !in labels);
        
    @("prefix")
    auto rep(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    @("prefix")
    auto repe(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    @("prefix")
    auto repz(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    @("prefix")
    auto repne(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    @("prefix")
    auto repnz(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    @("m8", "m8")
    auto movs(Mem!8 dst, Mem!8 src) => emit!0(0xa4, dst, src);
    @("m16", "m16")
    auto movs(Mem!16 dst, Mem!16 src) => emit!0(0xa5, dst, src);
    @("m32", "m32")
    auto movs(Mem!32 dst, Mem!32 src) => emit!0(0xa5, dst, src);
    @("m64", "m64")
    auto movs(Mem!64 dst, Mem!64 src) => emit!0(0xa5, dst, src);

    auto movsb() => emit!0(0xa4);
    auto movsw() => emit!0(0x66, 0xa5);
    auto movsd() => emit!0(0xa5);
    auto movsq() => emit!0(0x48, 0xa5);

    @("m8", "m8")
    auto cmps(Mem!8 dst, Mem!8 src) => emit!0(0xa6, dst, src);
    @("m16", "m16")
    auto cmps(Mem!16 dst, Mem!16 src) => emit!0(0xa7, dst, src);
    @("m32", "m32")
    auto cmps(Mem!32 dst, Mem!32 src) => emit!0(0xa7, dst, src);
    @("m64", "m64")
    auto cmps(Mem!64 dst, Mem!64 src) => emit!0(0xa7, dst, src);

    auto cmpsb() => emit!0(0xa6);
    auto cmpsw() => emit!0(0x66, 0xa7);
    auto cmpsd() => emit!0(0xa7);
    auto cmpsq() => emit!0(0x48, 0xa7);

    @("m8")
    auto scas(Mem!8 dst) => emit!0(0xae, dst);
    @("m16")
    auto scas(Mem!16 dst) => emit!0(0xaf, dst);
    @("m32")
    auto scas(Mem!32 dst) => emit!0(0xaf, dst);
    @("m64")
    auto scas(Mem!64 dst) => emit!0(0xaf, dst);

    auto scasb() => emit!0(0xae);
    auto scasw() => emit!0(0x66, 0xaf);
    auto scasd() => emit!0(0xaf);
    auto scasq() => emit!0(0x48, 0xaf);

    @("m8")
    auto lods(Mem!8 dst) => emit!0(0xac, dst);
    @("m16")
    auto lods(Mem!16 dst) => emit!0(0xad, dst);
    @("m32")
    auto lods(Mem!32 dst) => emit!0(0xad, dst);
    @("m64")
    auto lods(Mem!64 dst) => emit!0(0xad, dst);

    auto lodsb() => emit!0(0xac);
    auto lodsw() => emit!0(0x66, 0xad);
    auto lodsd() => emit!0(0xad);
    auto lodsq() => emit!0(0x48, 0xad);

    @("m8")
    auto stos(Mem!8 dst) => emit!0(0xaa, dst);
    @("m16")
    auto stos(Mem!16 dst) => emit!0(0xab, dst);
    @("m32")
    auto stos(Mem!32 dst) => emit!0(0xab, dst);
    @("m64")
    auto stos(Mem!64 dst) => emit!0(0xab, dst);

    auto stosb() => emit!0(0xaa);
    auto stosw() => emit!0(0x66, 0xab);
    auto stosd() => emit!0(0xab);
    auto stosq() => emit!0(0x48, 0xab);

    @("imm8")
    auto inal(ubyte imm8) => emit!0(0xe4, imm8);
    auto inal() => emit!0(0xec);

    @("imm8")
    auto _in(ubyte imm8) => emit!0(0xe5, imm8);
    auto _in() => emit!0(0xed);

    @("m8")
    auto ins(Mem!8 dst) => emit!0(0x6c, dst);
    @("m16")
    auto ins(Mem!16 dst) => emit!0(0x6d, dst);
    @("m32")
    auto ins(Mem!32 dst) => emit!0(0x6d, dst);

    auto insb() => emit!0(0x6c);
    auto insw() => emit!0(0x66, 0x6d);
    auto insd() => emit!0(0x6d);
    
    @("imm8")
    auto outal(ubyte imm8) => emit!0(0xe6, imm8);
    auto outal() => emit!0(0xee);

    @("imm8")
    auto _out(ubyte imm8) => emit!0(0xe7, imm8);
    auto _out() => emit!0(0xef);

    @("m8")
    auto outs(Mem!8 dst) => emit!0(0x6e, dst);
    @("m16")
    auto outs(Mem!16 dst) => emit!0(0x6f, dst);
    @("m32")
    auto outs(Mem!32 dst) => emit!0(0x6f, dst);

    auto outsb() => emit!0(0x6e);
    auto outsw() => emit!0(0x66, 0x6f);
    auto outsd() => emit!0(0x6f);

    @("rm8")
    auto seta(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x97, dst);
    @("rm8")
    auto setae(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x93, dst);
    @("rm8")
    auto setb(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x92, dst);
    @("rm8")
    auto setbe(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x96, dst);
    @("rm8")
    auto setc(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x92, dst);
    @("rm8")
    auto sete(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x94, dst);
    @("rm8")
    auto setg(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9f, dst);
    @("rm8")
    auto setge(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9d, dst);
    @("rm8")
    auto setl(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9c, dst);
    @("rm8")
    auto setle(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9e, dst);
    @("rm8")
    auto setna(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x96, dst);
    @("rm8")
    auto setnae(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x92, dst);
    @("rm8")
    auto setnb(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x93, dst);
    @("rm8")
    auto setnbe(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x97, dst);
    @("rm8")
    auto setnc(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x93, dst);
    @("rm8")
    auto setne(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x95, dst);
    @("rm8")
    auto setng(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9e, dst);
    @("rm8")
    auto setnge(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9c, dst);
    @("rm8")
    auto setnl(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9d, dst);
    @("rm8")
    auto setnle(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9f, dst);
    @("rm8")
    auto setno(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x91, dst);
    @("rm8")
    auto setnp(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9b, dst);
    @("rm8")
    auto setns(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x99, dst);
    @("rm8")
    auto setnz(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x95, dst);
    @("rm8")
    auto seto(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x90, dst);
    @("rm8")
    auto setp(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9a, dst);
    @("rm8")
    auto setpe(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9a, dst);
    @("rm8")
    auto setpo(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9b, dst);
    @("rm8")
    auto sets(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x98, dst);
    @("rm8")
    auto setz(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x94, dst);

    Mem!8 bytePtr(ARGS...)(ARGS args) => Mem!8(args);
    Mem!16 wordPtr(ARGS...)(ARGS args) => Mem!16(args);
    Mem!32 dwordPtr(ARGS...)(ARGS args) => Mem!32(args);
    Mem!64 qwordPtr(ARGS...)(ARGS args) => Mem!64(args);
    Mem!128 xmmwordPtr(ARGS...)(ARGS args) => Mem!128(args);
    Mem!256 ymmwordPtr(ARGS...)(ARGS args) => Mem!256(args);
    Mem!512 zmmwordPtr(ARGS...)(ARGS args) => Mem!512(args);
}

unittest
{
    Block!true block;
    with (block)
    {
        mov(eax, ecx);
        movsxd(rcx, eax);
        mov(ebx, 1);
        // TODO: pop and push are emitting REX but shouldn't
        pop(rbx);
        push(rcx);
        jl("a");
    label("a");
        popf();
        // Not supported in 64-bit
        //pusha();
        ret();
        retf(3);
        jmp("a");
        jb("a");
        setz(al);
        //aad(17);
        insb();
        outal();
        call(2);
        lock(add(eax, ebx));
        xacquire_lock(sub(si, di));
        movsb();
        // TODO: Make emittable instructions condiitonal?
        //daa();
        //das();
        //aaa();
        //pushcs();
        mov(eax, dwordPtr(ebx));
        // TODO: This is outputting 0x67 when it should output REX
        mov(eax, dwordPtr(rbx));
        //verr(si);
        stc();
        std();
        clc();
        wait();
        fwait();
        monitor();
        lfence();
        sfence();
        retf();
        test(al, bl);
        hlt();
        swapgs();
        inc(eax);
        dec(rax);
        dec(rdi);
        sub(rdi, 10);
        mul(esi);
        scasb();
        cmpsb();
        pause();
        iret();
        mov(esp, dwordPtr(rdx));
        pop(rsp);
        mov(rbp, rsp);
    }
    import tern.digest;
    import std.stdio;
    debug writeln(block.finalize().toHexString);
}

static foreach (S; __traits(getOverloads, Block!true, "andn", true))
    pragma(msg, S.stringof);