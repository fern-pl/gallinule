// Originally based on: https://github.com/philpax/djitt
module gallinule.x86;

import std.bitmanip;
import std.traits;
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
    if (isInstanceOf!(Address, SRC) && isInstanceOf!(Reg, DST))
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
    if (isInstanceOf!(Address, SRC) && isInstanceOf!(Address, DST))
{
    return generateModRM!OP(Reg!(TemplateArgsOf!(DST))(dst.register), Reg!(TemplateArgsOf!(SRC))(src.register));
}

ubyte[] generateModRM(ubyte OP, SRC, DST)(SRC src, DST dst)
    if (isInstanceOf!(Reg, SRC) && isInstanceOf!(Address, DST))
{
    return generateModRM!OP(dst, src);
}

/// This is simply used for constraining T to be an address or register of the given size(s).
enum valid(T, short SIZE) = is(T == Reg!SIZE) || is(T == Address!SIZE);
enum valid(T, short RS, short AS) = is(T == Reg!RS) || is(T == Address!AS);

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

    Addresses: Address!(SIZE)

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

public struct Address(short SIZE)
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

public enum OpCode : ushort
{
    CRIDVME,
    CRIDPVI,
    CRIDTSD,
    CRIDDE,
    CRIDPSE,
    CRIDPAE,
    CRIDMCE,
    CRIDPGE,
    CRIDPCE,
    CRIDOSFXSR,
    CRIDOSXMMEXCPT,
    CRIDUMIP,
    CRIDVMXE,
    CRIDSMXE,
    CRIDFSGSBASE,
    CRIDPCIDE,
    CRIDOSXSAVE,
    CRIDSMEP,
    CRIDSMAP,
    CRIDPKE,
    CRIDCET,
    CRIDPKS,
    CRIDUINTR,

    IDAVX512VL,
    IDAVX512BW,
    IDSHA,
    IDAVX512CD,
    IDAVX512ER,
    IDAVX512PF,
    IDPT,
    IDCLWB,
    IDCLFLUSHOPT,
    IDPCOMMIT,
    IDAVX512IFMA,
    IDSMAP,
    IDADX,
    IDRDSEED,
    IDAVX512DQ,
    IDAVX512F,
    IDPQE,
    IDRTM,
    IDINVPCID,
    IDERMS,
    IDBMI2,
    IDSMEP,
    IDFPDP,
    IDAVX2,
    IDHLE,
    IDBMI1,
    IDSGX,
    IDTSCADJ,
    IDFSGSBASE,
    
    IDPREFETCHWT1,
    IDAVX512VBMI,
    IDUMIP,
    IDPKU,
    IDAVX512VBMI2,
    IDCET,
    IDGFNI,
    IDVAES,
    IDVPCL,
    IDAVX512VNNI,
    IDAVX512BITALG,
    IDTME,
    IDAVX512VP,
    IDVA57,
    IDRDPID,
    IDSGXLC,
    
    IDAVX512QVNNIW,
    IDAVX512QFMA,
    IDPCONFIG,
    IDIBRSIBPB,
    IDSTIBP,

    IDSSE3,
    IDPCLMUL,
    IDDTES64,
    IDMON,
    IDDSCPL,
    IDVMX,
    IDSMX,
    IDEST,
    IDTM2,
    IDSSSE3,
    IDCID,
    IDSDBG,
    IDFMA,
    IDCX16,
    IDXTPR,
    IDPDCM,
    IDPCID,
    IDDCA,
    IDSSE41,
    IDSSE42,
    IDX2APIC,
    IDMOVBE,
    IDPOPCNT,
    IDTSCD,
    IDAES,
    IDXSAVE,
    IDOSXSAVE,
    IDAVX,
    IDF16C,
    IDRDRAND,
    IDHV,

    IDFPU,
    IDVME,
    IDDE,
    IDPSE,
    IDTSC,
    IDMSR,
    IDPAE,
    IDCX8,
    IDAPIC,
    IDSEP,
    IDMTRR,
    IDPGE,
    IDMCA,
    IDCMOV,
    IDPAT,
    IDPSE36,
    IDPSN,
    IDCLFL,
    IDDS,
    IDACPI,
    IDMMX,
    IDFXSR,
    IDSSE,
    IDSSE2,
    IDSS,
    IDHTT,
    IDTM,
    IDIA64,
    IDPBE,
    //
    PFADD,
    PFSUB,
    PFSUBR,
    PFMUL,

    PFCMPEQ,
    PFCMPGE,
    PFCMPGT,

    PF2ID,
    PI2FD,
    PF2IW,
    PI2FW,

    PFMAX,
    PFMIN,

    PFRCP,
    PFRSQRT,
    PFRCPIT1,
    PFRSQIT1,
    PFRCPIT2,

    PFACC,
    PFNACC,
    PFPNACC,
    PMULHRW,

    PAVGUSB,
    PSWAPD,

    FEMMS,
    //
    ICEBP,
    //
    PTWRITE,
    //
    CLWB,
    //
    CLFLUSHOPT,
    //
    STAC,
    CLAC,
    //
    ADC,
    ADCX,
    ADOX,
    //
    RDSEED,
    //
    BNDCL,
    BNDCU,
    BNDLDX,
    BNDSTX,
    BNDMK,
    BNDMOV,
    BOUND,
    //
    XEND,
    XABORT,
    XBEGIN,
    XTEST,
    //
    INVPCID,
    //
    XACQUIRE,
    XRELEASE,
    //
    TZCNT,
    LZCNT,
    ANDN,
    //
    ECREATE,
    EINIT,
    EREMOVE,
    EDBGRD,
    EDBGWR,
    EEXTEND,
    ELDB,
    ELDU,
    EBLOCK,
    EPA,
    EWB,
    ETRACK,
    EAUG,
    EMODPR,
    EMODT,
    ERDINFO,
    ETRACKC,
    ELDBC,
    ELDUC,
    
    EREPORT,
    EGETKEY,
    EENTER,
    EEXIT,
    EACCEPT,
    EMODPE,
    EACCEPTCOPY,
    EDECCSSA,

    EDECVIRTCHILD,
    EINCVIRTCHILD,
    ESETCONTEXT,
    //
    MONITOR,
    MWAIT,
    //
    INVVPID,
    INVEPT,

    VMCALL,
    VMFUNC,
    VMCLEAR,
    VMLAUNCH,
    VMRESUME,
    VMXOFF,
    VMXON,

    VMWRITE,
    VMREAD,

    VMPTRST,
    VMPTRLD,
    //
    CAPABILITIES,
    ENTERACCS,
    EXITAC,
    SENTER,
    SEXIT,
    PARAMETERS,
    SMCTRL,
    WAKEUP,
    //
    CMPXCHG16B,
    //
    POPCNT,
    //
    XGETBV,
    XSETBV,

    XRSTOR,
    XSAVE,

    XRSTORS,
    XSAVES,

    XSAVEOPT,
    XSAVEC,
    //
    RDRAND,
    //
    FABS,
    FCHS,

    FCLEX,
    FNCLEX,

    FADD,
    FADDP,
    FIADD,

    FBLD,
    FBSTP,

    FCOM,
    FCOMP,
    FCOMPP,

    FCOMI,
    FCOMIP,
    FUCOMI,
    FUCOMIP,
    
    FICOM,
    FICOMP,

    FUCOM,
    FUCOMP,
    FUCOMPP,

    FTST,

    F2XM1,
    FYL2X,
    FYL2XP1,

    FCOS,
    FSIN,
    FSINCOS,
    FSQRT,

    FPTAN,
    FPATAN,
    FPREM,
    FPREM1,

    FDECSTP,
    FINCSTP,

    FILD,
    FIST,
    FISTP,
    FISTTP,

    FLDCW,
    FSTCW,
    FNSTCW,

    FLDENV,
    FSTENV,
    FNSTENV,

    FSTSW,
    FNSTSW,

    FLD,
    FLD1,
    FLDL2T,
    FLDL2E,
    FLDPI,
    FLDLG2,
    FLDLN2,
    FLDZ,

    FST,
    FSTP,

    FDIV,
    FDIVP,
    FIDIV,

    FDIVR,
    FDIVRP,
    FIDIVR,

    FSCALE,
    FRNDINT,
    FEXAM,
    FFREE,
    FXCH,
    FXTRACT,

    FNOP,
    FNINIT,
    FINIT,

    FSAVE,
    FNSAVE,

    FRSTOR,
    FXSAVE,

    FXRSTOR,

    FMUL,
    FMULP,
    FIMUL,

    FSUB,
    FSUBP,
    FISUB,

    FSUBR,
    FSUBRP,
    FISUBR,

    FCMOVCC,
    //
    RDMSR,
    WRMSR,
    //
    CMPXCHG8B,
    //
    SYSENTER,
    SYSEXITC,
    SYSEXIT,
    //
    CMOVCC,
    //
    CLFLUSH,
    //
    HRESET,
    //
    INCSSPD,
    INCSSPQ,
    CLRSSBSY,
    SETSSBSY,

    RDSSPD,
    RDSSPQ,
    WRSSD,
    WRSSQ,
    WRUSSD,
    WRUSSQ,

    RSTORSSP,
    SAVEPREVSSP,

    ENDBR32,
    ENDBR64,

    RDFSBASE,
    RDGSBASE,

    WRFSBASE,
    WRGSBASE,
    //
    RDPID,
    //
    WRPKRU,
    RDPKRU,
    //
    TESTUI,
    STUI,
    CLUI,
    UIRET,
    SENDUIPI,
    //
    UMWAIT,
    UMONITOR,
    TPAUSE,
    //
    CLDEMOTE,
    //
    XRESLDTRK,
    XSUSLDTRK,
    //
    SERIALIZE,
    //
    PCONFIG,
    //
    RDPMC,
    //
    WBINVD,
    WBNOINVD,

    INVD,

    LGDT,
    SGDT,

    LLDT,
    SLDT,

    LIDT,
    SIDT,

    LMSW,
    SMSW,
    //
    INVLPG,
    //
    SAHF,
    LAHF,
    //
    SARX,
    SHLX,
    SHRX,
    //
    MOVQ,
    MOVD,
    //
    ADDPD,
    ADDPS,
    ADDSS,
    ADDSD,
    //
    LFENCE,
    SFENCE,
    MFENCE,
    //
    ADDSUBPS,
    ADDSUBPD,
    //
    VADDPD,
    VADDPS,
    VADDSD,
    VADDSS,

    VADDSUBPD,
    VADDSUBPS,

    VMOVQ,
    VMOVD,
    //
    AESDEC,
    VAESDEC,

    AESDEC128KL,
    AESDEC256KL,

    AESDECLAST,
    VAESDECLAST,

    AESDECWIDE128KL,
    AESDECWIDE256KL,

    AESENC,
    VAESENC,

    AESENC128KL,
    AESENC256KL,

    AESENCLAST,
    VAESENCLAST,

    AESENCWIDE128KL,
    AESENCWIDE256KL,

    AESIMC,
    VAESIMC,

    AESKEYGENASSIST,
    VAESKEYGENASSIST,
    //
    SHA1MSG1,
    SHA1MSG2,
    SHA1NEXTE,

    SHA256MSG1,
    SHA1RNDS4,
    SHA256RNDS2,
    //
    //NOT_TAKEN,
    //TAKEN,
    CRC32,

    ENDQCMD,

    CMPXCHG,

    AAA,
    AAD,
    AAM,
    AAS,
    ADD,
    AND,

    ARPL,

    BSF,
    BSR,
    BSWAP,
    BT,
    BTC,
    BTR,
    BTS,
    
    CMP,

    CWD,
    CDQ,
    CQO,

    CBW,
    CWDE,
    CDQE,

    CPUID,

    CLC,
    CLD,
    CLI,
    CLTS,
    CMC,

    DEC,

    INT,
    INTO,
    UD,
    IRET,

    INC,

    HLT,
    PAUSE,
    SWAPGS,

    LOCK,

    WAIT,
    FWAIT,

    SYSRETC,
    SYSRET,
    SYSCALL,
    RSM,

    LEAVE,
    ENTER,

    LEA,
    LDS,
    LSS,
    LES,
    LFS,
    LGS,
    LSL,
    
    LTR,
    STR,

    NEG,
    NOP,
    NOT,

    RET,
    RETF,

    STC,
    STD,
    STI,

    SUB,
    SBB,

    XOR,
    OR,

    SAL,
    SAR,
    SHL,
    SHR,

    RCL,
    RCR,
    ROL,
    ROR,

    VERR,
    VERW,

    TEST,

    POP,
    POPDS,
    POPES,
    POPSS,
    POPFS,
    POPGS,
    POPA,
    POPF,

    PUSH,
    PUSHCS,
    PUSHSS,
    PUSHDS,
    PUSHES,
    PUSHFS,
    PUSHGS,
    PUSHA,
    PUSHF,

    XADD,
    XCHG,
    XLAT,
    XLATB,

    LAR,

    DAA,
    DAS,

    MUL,
    IMUL,
    DIV,
    IDIV,

    MOV,
    MOVSX,
    MOVSXD,
    MOVZX,
    MOVS,
    MOVSB,
    MOVSW,
    MOVSD,
    MOVSQ,

    CALL,
    LOOPCC,
    JMP,
    JCC,
    REPCC,

    CMPS,
    CMPSB,
    CMPSW,
    CMPSD,
    CMPSQ,

    SCAS,
    SCASB,
    SCASW,
    SCASD,
    SCASQ,

    LODS,
    LODSB,
    LODSW,
    LODSD,
    LODSQ,

    STOS,
    STOSB,
    STOSW,
    STOSD,
    STOSQ,

    IN,
    INS,
    INSB,
    INSW,
    INSD,

    OUT,
    OUTS,
    OUTSB,
    OUTSW,
    OUTSD,

    SETCC
}

public enum TypeModifiers : ubyte
{
    FLOAT = 1 << 0,
    POINTER = 1 << 1,
    ARRAY = 1 << 2,
    // Specially defined because vectors and strings get register priority regardless of usage :-)
    VECTOR = 1 << 3,
    BYTE = 1 << 4,
    WORD = 1 << 5,
    DWORD = 1 << 6,
    QWORD = 1 << 7,

    STRING = VECTOR | ARRAY,
    INTEGRAL_MASK = 0b00001111
}

public enum Details
{
    // PUSHA, POPA, RET and CALL require special parsing
    READ1 = 1 << 0,
    READ2 = 1 << 1,
    READ3 = 1 << 2,
    WRITE1 = 1 << 3,
    WRITE2 = 1 << 4,
    WRITE3 = 1 << 5,

    POLLUTE_AX = 1 << 6,
    POLLUTE_BX = 1 << 7,
    POLLUTE_CX = 1 << 8,
    POLLUTE_DX = 1 << 9,

    // These are not opcode defined
    GREATER = 1 << 10,
    GREATEREQ = 1 << 11,
    LESSER = 1 << 12,
    LESSEREQ = 1 << 13,
    EQUAL = 1 << 14,
    NEQUAL = 1 << 15,
    CARRY = 1 << 16,
    NCARRY = 1 << 17,
    SIGN = 1 << 18,
    NSIGN = 1 << 19,
    ZERO = 1 << 20,
    NZERO = 1 << 21,
    TAKEN = 1 << 22,
    NOT_TAKEN = 1 << 23
}

public enum MarkerKind : ubyte
{
    LITERAL,
    ALLOCATION,
    REGISTER
}

public struct Marker
{
public:
final:
    MarkerKind kind;
    ptrdiff_t size;
    union
    {
        struct //asAllocation
        {
            ubyte segment = ds;
            uint offset;
            short baseSize;
            // Will cause problems? Extended registers, dunno
            ubyte baseIndex = 255;
        }

        struct //asRegister
        {
            ubyte index;
            bool extended;
        }

        struct //asLiteral
        {
            union
            {
                ubyte b;
                ushort w;
                uint d;
                ulong q;
            }
        }
    }

    this(size_t size, uint offset, ubyte segment = ds, short baseSize = 8, ubyte baseIndex = 255)
    {
        this.kind = MarkerKind.ALLOCATION;
        this.size = size;
        this.segment = segment;
        this.offset = offset;
        this.baseSize = baseSize;
        this.baseIndex = baseIndex;
    }

    this(size_t size, ubyte index, bool extended)
    {
        this.kind = MarkerKind.REGISTER;
        this.size = size;
        this.index = index;
        this.extended = extended;
    }

    this(T)(T val)
    {
        this.kind = MarkerKind.LITERAL;
        this.size = T.sizeof;

        /* static if (is(T : U*, U))
        {
            type = Type(0, Modifiers.MEMORY_LITERAL);
            ptr = cast(void*)val;
        }
        else static if (is(T == string))
            name = val; */
        static if (T.sizeof == 1)
            b = cast(ubyte)val;
        else static if (T.sizeof == 2)
            w = cast(ushort)val;
        else static if (T.sizeof == 4)
            d = cast(uint)val;
        else static if (T.sizeof == 8)
            q = cast(ulong)val;
    }

    T as(T)()
    {
        static if (isInstanceOf!(Reg, T))
        if (kind == MarkerKind.REGISTER)
            return T(index, extended);

        static if (isInstanceOf!(Address, T))
        if (kind == MarkerKind.ALLOCATION)
        {
            if (baseIndex != 255)
            {
                T ret = T(offset, segment);
                ret.register = baseIndex;
                ret.size = cast(short)(baseSize * 8);
                return ret;
            }
            else
                return T(offset, segment);
        }

        assert(0, "Attempted to convert a marker not of kind REGISTER or ALLOCATION to a type!");
    }
}

public struct Variable
{
public:
final:
    string name;
    TypeModifiers modifiers;
    size_t size;
    Marker[] markers;
    int score;

    ref Marker firstMark()
    {
        assert(markers.length > 0, "Attempted to retrieve the first mark of an unmarked variable!");
        return markers[0];
    }

    ref Marker lastMark()
    {
        assert(markers.length > 0, "Attempted to retrieve the last mark of an unmarked variable!");
        return markers[$-1];
    }
}

public struct Instruction
{
public:
final:
    OpCode opcode;
    Variable[] operands;
    Details details;
    int score;

    bool markFormat(string fmt)
    {
        if (fmt.length > operands.length)
            return false;

        foreach (i, c; fmt)
        {
            switch (c)
            {
                case 'l':
                    if (firstMark(i).kind != MarkerKind.LITERAL)
                        return false;
                    break;
                case 'm':
                    if (firstMark(i).kind != MarkerKind.ALLOCATION)
                        return false;
                    break;
                case 'r':
                    if (firstMark(i).kind != MarkerKind.REGISTER)
                        return false;
                    break;
                case 'n':
                    if (firstMark(i).kind == MarkerKind.LITERAL)
                        return false;
                    break;
                case '.':
                    if (fmt.length != operands.length)
                        return false;
                    break;
                default:
                    assert(0, "Invalid character in mask format comparison '"~fmt~"'!");
            }
        }
        return true;
    }

    ref Marker firstMark(size_t index)
    {
        assert(operands.length > index, "Attempted to retrieve first mark of an out of bounds operand!");
        return operands[index].firstMark;
    }

    ref Marker lastMark(size_t index)
    {
        assert(operands.length > index, "Attempted to retrieve last mark of an out of bounds operand!");
        return operands[index].lastMark;
    }

    this(OpCode opcode, Variable[] operands...)
    {
        Details pollute(string fmt) pure
        {
            Details ret;
            foreach (i, c; fmt)
            {
                switch (c)
                {
                    case 'r':
                        if (i == 0)
                            ret |= Details.READ1;
                        else if (i == 1)
                            ret |= Details.READ2;
                        else if (i == 2)
                            ret |= Details.READ3;
                        break;
                    case 'w':
                        if (i == 0)
                            ret |= Details.WRITE1;
                        else if (i == 1)
                            ret |= Details.WRITE2;
                        else if (i == 2)
                            ret |= Details.WRITE3;
                        break;
                    case 'a':
                        ret |= Details.POLLUTE_AX;
                        break;
                    case 'b':
                        ret |= Details.POLLUTE_BX;
                        break;
                    case 'c':
                        ret |= Details.POLLUTE_CX;
                        break;
                    case 'd':
                        ret |= Details.POLLUTE_DX;
                        break;
                }
            }
            return ret;
        }

        this.opcode = opcode;
        this.operands = operands;

        with (OpCode) switch (opcode)
        {
            case ADD:
            case SUB:
            case ROL:
            case ROR:
            case RCL:
            case SHL:
            case SHR:
            case SAR:
            case SAL:
            case XOR:
            case OR:
            case AND:
            case BT:
            case BTC:
            case BTR:
            case BTS:
            case TEST:
                if (operands.length == 1)
                    details = pollute("ra");
                else
                    details = pollute("rr");
            case LTR:
            case INC:
            case DEC:
            case BSWAP:
            case SETCC:
            case PUSH:
            case NOT:
            case NEG:
                details = pollute("r");
            case NOP:
                if (operands.length == 1)
                    details = pollute("r");
                break;
            case STR:
            case POP:
                details = pollute("w");
            case IDIV:
            case DIV:
            case MUL:
                details = pollute("rad");
            case IMUL:
                if (operands.length == 1)
                    details = pollute("rad");
                else
                    details = pollute("rr");
            case MOVSX:
            case MOVSXD:
            case MOVZX:
            case MOV:
            case LEA:
            case LDS:
            case LSS:
            case LES:
            case LFS:
            case LGS:
            case LSL:
            case CMOV:
            case LZCNT:
            case TZCNT:
            case BSF:
            case BSR:
                details = pollute("wr");
            case CRIDCET:
            case CRIDDE:
            case CRIDFSGSBASE:
            case CRIDMCE:
            case CRIDOSFXSR:
            case CRIDOSXMMEXCPT:
            case CRIDOSXSAVE:
            case CRIDPAE:
            case CRIDPCE:
            case CRIDPCIDE:
            case CRIDPGE:
            case CRIDPKE:
            case CRIDPKS:
            case CRIDPSE:
            case CRIDPVI:
            case CRIDSMAP:
            case CRIDSMEP:
            case CRIDSMXE:
            case CRIDTSD:
            case CRIDUMIP:
            case CRIDUINTR:
            case CRIDVME:
            case CRIDVMXE:
            case ECREATE:
            case EINIT:
            case EREMOVE:
            case EDBGRD:
            case EDBGWR:
            case EEXTEND:
            case ELDB:
            case ELDU:
            case EBLOCK:
            case EPA:
            case EWB:
            case ETRACK:
            case EAUG:
            case EMODPR:
            case EMODT:
            case ERDINFO:
            case ETRACKC:
            case ELDBC:
            case ELDUC:
            case EREPORT:
            case EGETKEY:
            case EENTER:
            case EEXIT:
            case EACCEPT:
            case EMODPE:
            case EACCEPTCOPY:
            case EDECCSSA:
            case EDECVIRTCHILD:
            case EINCVIRTCHILD:
            case ESETCONTEXT:
            case CAPABILITIES:
            case ENTERACCS:
            case EXITAC:
            case SENTER:
            case SEXIT:
            case PARAMETERS:
            case SMCTRL:
            case WAKEUP:
            case RDPKRU:
            case WRPKRU:
            case LAHF:
            case SAHF:
            case CPUID:
                details = pollute("a");
                break;
            case IDACPI:
            case IDADX:
            case IDAES:
            case IDAPIC:
            case IDAVX:
            case IDAVX2:
            case IDAVX512BITALG:
            case IDAVX512BW:
            case IDAVX512CD:
            case IDAVX512DQ:
            case IDAVX512ER:
            case IDAVX512F:
            case IDAVX512IFMA:
            case IDAVX512PF:
            case IDAVX512QFMA:
            case IDAVX512QVNNIW:
            case IDAVX512VBMI:
            case IDAVX512VBMI2:
            case IDAVX512VL:
            case IDAVX512VNNI:
            case IDAVX512VP:
            case IDBMI1:
            case IDBMI2:
            case IDCET:
            case IDCID:
            case IDCLFL:
            case IDCLFLUSHOPT:
            case IDCLWB:
            case IDCMOV:
            case IDCX16:
            case IDCX8:
            case IDDCA:
            case IDDTES64:
            case IDDE:
            case IDDS:
            case IDDSCPL:
            case IDERMS:
            case IDEST:
            case IDF16C:
            case IDFMA:
            case IDFPDP:
            case IDFPU:
            case IDFSGSBASE:
            case IDFXSR:
            case IDGFNI:
            case IDHLE:
            case IDHTT:
            case IDHV:
            case IDIA64:
            case IDIBRSIBPB:
            case IDINVPCID:
            case IDMCA:
            case IDMMX:
            case IDMON:
            case IDMOVBE:
            case IDMSR:
            case IDMTRR:
            case IDOSXSAVE:
            case IDPAE:
            case IDPAT:
            case IDPBE:
            case IDPCID:
            case IDPCLMUL:
            case IDPCOMMIT:
            case IDPCONFIG:
            case IDPDCM:
            case IDPGE:
            case IDPKU:
            case IDPOPCNT:
            case IDPQE:
            case IDPREFETCHWT1:
            case IDPSE:
            case IDPSE36:
            case IDPSN:
            case IDPT:
            case IDRDPID:
            case IDRDRAND:
            case IDRDSEED:
            case IDRTM:
            case IDSDBG:
            case IDSEP:
            case IDSGX:
            case IDSGXLC:
            case IDSHA:
            case IDSMAP:
            case IDSMEP:
            case IDSMX:
            case IDSS:
            case IDSSE:
            case IDSSE2:
            case IDSSE3:
            case IDSSE41:
            case IDSSE42:
            case IDSSSE3:
            case IDSTIBP:
            case IDTM:
            case IDTM2:
            case IDTME:
            case IDTSC:
            case IDTSCADJ:
            case IDTSCD:
            case IDUMIP:
            case IDVA57:
            case IDVAES:
            case IDVME:
            case IDVMX:
            case IDVPCL:
            case IDX2APIC:
            case IDXSAVE:
            case IDXTPR:
            case RDMSR:
            case WRMSR:
            case RDPMC:
                details = pollute("abcd");
                break;
            case RET:
            case INT:
            case STAC:
            case STC:
            case STD:
            case STI:
            case CLAC:
            case CLC:
            case CLD:
            case CLI:
            case SYSCALL:
            case SYSENTER:
            case SYSEXIT:
            case SYSEXITC:
            case INTO:
            case RETF:
                break;
            default:
                assert(0, "Unimplemented instruction opcode!");
                break;
        }
    }
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

enum cr0 = Reg!(-1)(0);
enum cr2 = Reg!(-1)(2);
enum cr3 = Reg!(-1)(3);
enum cr4 = Reg!(-1)(4);

enum dr0 = Reg!(-2)(0);
enum dr1 = Reg!(-2)(1);
enum dr2 = Reg!(-2)(2);
enum dr3 = Reg!(-2)(3);
enum dr6 = Reg!(-2)(6);
enum dr7 = Reg!(-2)(7);

// ST registers aren't real registers, the FPU uses a stack
enum st0 = Reg!(-3)(0);
enum st1 = Reg!(-3)(1);
enum st2 = Reg!(-3)(2);
enum st3 = Reg!(-3)(3);
enum st4 = Reg!(-3)(4);
enum st5 = Reg!(-3)(5);
enum st6 = Reg!(-3)(6);
enum st7 = Reg!(-3)(7);

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
                
                static if (isInstanceOf!(Address, SRC))
                {
                    if ((X64 && src.size != 64) || src.size != 32)
                        buffer = 0x67~buffer;
                }

                static if (isInstanceOf!(Address, DST))
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
                        we = is(SRC == Reg!64);
                    b = src.index >= 8;
                }
                else static if (isInstanceOf!(Address, SRC))
                {
                    static if (SELECTOR == VEXI)
                        we = is(SRC == Address!64);
                    b = src.register >= 8;
                }
                
                static if (isInstanceOf!(Reg, DST))
                {
                    static if (SELECTOR == VEXI)
                        we = is(DST == Reg!64);
                    r = dst.index >= 8;
                }
                else static if (isInstanceOf!(Address, DST))
                {
                    static if (SELECTOR == VEXI)
                        we = is(DST == Address!64);
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
                
                static if (isInstanceOf!(Address, SRC))
                {
                    if ((X64 && src.size != 64) || src.size != 32)
                        buffer = 0x67~buffer;
                }

                static if (isInstanceOf!(Address, DST))
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
                else static if (is(typeof(arg) == long))
                    buffer ~= (cast(ubyte*)&arg)[0..uint.sizeof];
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
    
    auto emit(Instruction instr)
    {
        // Should check to make sure the instruction is valid,
        // conditional instructions need to have an actual condition flag.
        //assert(!instr.details.hasFlag(Details.ILLEGAL), "Invalid instruction, are you missing a condition?");

        with (OpCode) switch (instr.opcode)
        {
            case CRIDVME:
                enum ofn = "cridvme";
                mixin(ofn~"();");
                break;
            case CRIDPVI:
                enum ofn = "cridpvi";
                mixin(ofn~"();");
                break;
            case CRIDTSD:
                enum ofn = "cridtsd";
                mixin(ofn~"();");
                break;
            case CRIDDE:
                enum ofn = "cridde";
                mixin(ofn~"();");
                break;
            case CRIDPSE:
                enum ofn = "cridpse";
                mixin(ofn~"();");
                break;
            case CRIDPAE:
                enum ofn = "cridpae";
                mixin(ofn~"();");
                break;
            case CRIDMCE:
                enum ofn = "cridmce";
                mixin(ofn~"();");
                break;
            case CRIDPGE:
                enum ofn = "cridpge";
                mixin(ofn~"();");
                break;
            case CRIDPCE:
                enum ofn = "cridpce";
                mixin(ofn~"();");
                break;
            case CRIDOSFXSR:
                enum ofn = "cridosfxsr";
                mixin(ofn~"();");
                break;
            case CRIDOSXMMEXCPT:
                enum ofn = "cridosxmmexcpt";
                mixin(ofn~"();");
                break;
            case CRIDUMIP:
                enum ofn = "cridumip";
                mixin(ofn~"();");
                break;
            case CRIDVMXE:
                enum ofn = "cridvmxe";
                mixin(ofn~"();");
                break;
            case CRIDSMXE:
                enum ofn = "cridsmxe";
                mixin(ofn~"();");
                break;
            case CRIDFSGSBASE:
                enum ofn = "cridfsgsbase";
                mixin(ofn~"();");
                break;
            case CRIDPCIDE:
                enum ofn = "cridpcide";
                mixin(ofn~"();");
                break;
            case CRIDOSXSAVE:
                enum ofn = "cridosxsave";
                mixin(ofn~"();");
                break;
            case CRIDSMEP:
                enum ofn = "cridsmep";
                mixin(ofn~"();");
                break;
            case CRIDSMAP:
                enum ofn = "cridsmap";
                mixin(ofn~"();");
                break;
            case CRIDPKE:
                enum ofn = "cridpke";
                mixin(ofn~"();");
                break;
            case CRIDCET:
                enum ofn = "cridcet";
                mixin(ofn~"();");
                break;
            case CRIDPKS:
                enum ofn = "cridpks";
                mixin(ofn~"();");
                break;
            case CRIDUINTR:
                enum ofn = "criduintr";
                mixin(ofn~"();");
                break;
            case IDAVX512VL:
                enum ofn = "idavx512vl";
                mixin(ofn~"();");
                break;
            case IDAVX512BW:
                enum ofn = "idavx512bw";
                mixin(ofn~"();");
                break;
            case IDSHA:
                enum ofn = "idsha";
                mixin(ofn~"();");
                break;
            case IDAVX512CD:
                enum ofn = "idavx512cd";
                mixin(ofn~"();");
                break;
            case IDAVX512ER:
                enum ofn = "idavx512er";
                mixin(ofn~"();");
                break;
            case IDAVX512PF:
                enum ofn = "idavx512pf";
                mixin(ofn~"();");
                break;
            case IDPT:
                enum ofn = "idpt";
                mixin(ofn~"();");
                break;
            case IDCLWB:
                enum ofn = "idclwb";
                mixin(ofn~"();");
                break;
            case IDCLFLUSHOPT:
                enum ofn = "idclflushopt";
                mixin(ofn~"();");
                break;
            case IDPCOMMIT:
                enum ofn = "idpcommit";
                mixin(ofn~"();");
                break;
            case IDAVX512IFMA:
                enum ofn = "idavx512ifma";
                mixin(ofn~"();");
                break;
            case IDSMAP:
                enum ofn = "idsmap";
                mixin(ofn~"();");
                break;
            case IDADX:
                enum ofn = "idadx";
                mixin(ofn~"();");
                break;
            case IDRDSEED:
                enum ofn = "idrdseed";
                mixin(ofn~"();");
                break;
            case IDAVX512DQ:
                enum ofn = "idavx512dq";
                mixin(ofn~"();");
                break;
            case IDAVX512F:
                enum ofn = "idavx512f";
                mixin(ofn~"();");
                break;
            case IDPQE:
                enum ofn = "idpqe";
                mixin(ofn~"();");
                break;
            case IDRTM:
                enum ofn = "idrtm";
                mixin(ofn~"();");
                break;
            case IDINVPCID:
                enum ofn = "idinvpcid";
                mixin(ofn~"();");
                break;
            case IDERMS:
                enum ofn = "iderms";
                mixin(ofn~"();");
                break;
            case IDBMI2:
                enum ofn = "idbmi2";
                mixin(ofn~"();");
                break;
            case IDSMEP:
                enum ofn = "idsmep";
                mixin(ofn~"();");
                break;
            case IDFPDP:
                enum ofn = "idfpdp";
                mixin(ofn~"();");
                break;
            case IDAVX2:
                enum ofn = "idavx2";
                mixin(ofn~"();");
                break;
            case IDHLE:
                enum ofn = "idhle";
                mixin(ofn~"();");
                break;
            case IDBMI1:
                enum ofn = "idbmi1";
                mixin(ofn~"();");
                break;
            case IDSGX:
                enum ofn = "idsgx";
                mixin(ofn~"();");
                break;
            case IDTSCADJ:
                enum ofn = "idtscadj";
                mixin(ofn~"();");
                break;
            case IDFSGSBASE:
                enum ofn = "idfsgsbase";
                mixin(ofn~"();");
                break;
            case IDPREFETCHWT1:
                enum ofn = "idprefetchwt1";
                mixin(ofn~"();");
                break;
            case IDAVX512VBMI:
                enum ofn = "idavx512vbmi";
                mixin(ofn~"();");
                break;
            case IDUMIP:
                enum ofn = "idumip";
                mixin(ofn~"();");
                break;
            case IDPKU:
                enum ofn = "idpku";
                mixin(ofn~"();");
                break;
            case IDAVX512VBMI2:
                enum ofn = "idavx512vbmi2";
                mixin(ofn~"();");
                break;
            case IDCET:
                enum ofn = "idcet";
                mixin(ofn~"();");
                break;
            case IDGFNI:
                enum ofn = "idgfni";
                mixin(ofn~"();");
                break;
            case IDVAES:
                enum ofn = "idvaes";
                mixin(ofn~"();");
                break;
            case IDVPCL:
                enum ofn = "idvpcl";
                mixin(ofn~"();");
                break;
            case IDAVX512VNNI:
                enum ofn = "idavx512vnni";
                mixin(ofn~"();");
                break;
            case IDAVX512BITALG:
                enum ofn = "idavx512bitalg";
                mixin(ofn~"();");
                break;
            case IDTME:
                enum ofn = "idtme";
                mixin(ofn~"();");
                break;
            case IDAVX512VP:
                enum ofn = "idavx512vp";
                mixin(ofn~"();");
                break;
            case IDVA57:
                enum ofn = "idva57";
                mixin(ofn~"();");
                break;
            case IDRDPID:
                enum ofn = "idrdpid";
                mixin(ofn~"();");
                break;
            case IDSGXLC:
                enum ofn = "idsgxlc";
                mixin(ofn~"();");
                break;
            case IDAVX512QVNNIW:
                enum ofn = "idavx512qvnniw";
                mixin(ofn~"();");
                break;
            case IDAVX512QFMA:
                enum ofn = "idavx512qfma";
                mixin(ofn~"();");
                break;
            case IDPCONFIG:
                enum ofn = "idpconfig";
                mixin(ofn~"();");
                break;
            case IDIBRSIBPB:
                enum ofn = "idibrsibpb";
                mixin(ofn~"();");
                break;
            case IDSTIBP:
                enum ofn = "idstibp";
                mixin(ofn~"();");
                break;
            case IDSSE3:
                enum ofn = "idsse3";
                mixin(ofn~"();");
                break;
            case IDPCLMUL:
                enum ofn = "idpclmul";
                mixin(ofn~"();");
                break;
            case IDDTES64:
                enum ofn = "iddtes64";
                mixin(ofn~"();");
                break;
            case IDMON:
                enum ofn = "idmon";
                mixin(ofn~"();");
                break;
            case IDDSCPL:
                enum ofn = "iddscpl";
                mixin(ofn~"();");
                break;
            case IDVMX:
                enum ofn = "idvmx";
                mixin(ofn~"();");
                break;
            case IDSMX:
                enum ofn = "idsmx";
                mixin(ofn~"();");
                break;
            case IDEST:
                enum ofn = "idest";
                mixin(ofn~"();");
                break;
            case IDTM2:
                enum ofn = "idtm2";
                mixin(ofn~"();");
                break;
            case IDSSSE3:
                enum ofn = "idssse3";
                mixin(ofn~"();");
                break;
            case IDCID:
                enum ofn = "idcid";
                mixin(ofn~"();");
                break;
            case IDSDBG:
                enum ofn = "idsdbg";
                mixin(ofn~"();");
                break;
            case IDFMA:
                enum ofn = "idfma";
                mixin(ofn~"();");
                break;
            case IDCX16:
                enum ofn = "idcx16";
                mixin(ofn~"();");
                break;
            case IDXTPR:
                enum ofn = "idxtpr";
                mixin(ofn~"();");
                break;
            case IDPDCM:
                enum ofn = "idpdcm";
                mixin(ofn~"();");
                break;
            case IDPCID:
                enum ofn = "idpcid";
                mixin(ofn~"();");
                break;
            case IDDCA:
                enum ofn = "iddca";
                mixin(ofn~"();");
                break;
            case IDSSE41:
                enum ofn = "idsse41";
                mixin(ofn~"();");
                break;
            case IDSSE42:
                enum ofn = "idsse42";
                mixin(ofn~"();");
                break;
            case IDX2APIC:
                enum ofn = "idx2apic";
                mixin(ofn~"();");
                break;
            case IDMOVBE:
                enum ofn = "idmovbe";
                mixin(ofn~"();");
                break;
            case IDPOPCNT:
                enum ofn = "idpopcnt";
                mixin(ofn~"();");
                break;
            case IDTSCD:
                enum ofn = "idtscd";
                mixin(ofn~"();");
                break;
            case IDAES:
                enum ofn = "idaes";
                mixin(ofn~"();");
                break;
            case IDXSAVE:
                enum ofn = "idxsave";
                mixin(ofn~"();");
                break;
            case IDOSXSAVE:
                enum ofn = "idosxsave";
                mixin(ofn~"();");
                break;
            case IDAVX:
                enum ofn = "idavx";
                mixin(ofn~"();");
                break;
            case IDF16C:
                enum ofn = "idf16c";
                mixin(ofn~"();");
                break;
            case IDRDRAND:
                enum ofn = "idrdrand";
                mixin(ofn~"();");
                break;
            case IDHV:
                enum ofn = "idhv";
                mixin(ofn~"();");
                break;
            case IDFPU:
                enum ofn = "idfpu";
                mixin(ofn~"();");
                break;
            case IDVME:
                enum ofn = "idvme";
                mixin(ofn~"();");
                break;
            case IDDE:
                enum ofn = "idde";
                mixin(ofn~"();");
                break;
            case IDPSE:
                enum ofn = "idpse";
                mixin(ofn~"();");
                break;
            case IDTSC:
                enum ofn = "idtsc";
                mixin(ofn~"();");
                break;
            case IDMSR:
                enum ofn = "idmsr";
                mixin(ofn~"();");
                break;
            case IDPAE:
                enum ofn = "idpae";
                mixin(ofn~"();");
                break;
            case IDCX8:
                enum ofn = "idcx8";
                mixin(ofn~"();");
                break;
            case IDAPIC:
                enum ofn = "idapic";
                mixin(ofn~"();");
                break;
            case IDSEP:
                enum ofn = "idsep";
                mixin(ofn~"();");
                break;
            case IDMTRR:
                enum ofn = "idmtrr";
                mixin(ofn~"();");
                break;
            case IDPGE:
                enum ofn = "idpge";
                mixin(ofn~"();");
                break;
            case IDMCA:
                enum ofn = "idmca";
                mixin(ofn~"();");
                break;
            case IDCMOV:
                enum ofn = "idcmov";
                mixin(ofn~"();");
                break;
            case IDPAT:
                enum ofn = "idpat";
                mixin(ofn~"();");
                break;
            case IDPSE36:
                enum ofn = "idpse36";
                mixin(ofn~"();");
                break;
            case IDPSN:
                enum ofn = "idpsn";
                mixin(ofn~"();");
                break;
            case IDCLFL:
                enum ofn = "idclfl";
                mixin(ofn~"();");
                break;
            case IDDS:
                enum ofn = "idds";
                mixin(ofn~"();");
                break;
            case IDACPI:
                enum ofn = "idacpi";
                mixin(ofn~"();");
                break;
            case IDMMX:
                enum ofn = "idmmx";
                mixin(ofn~"();");
                break;
            case IDFXSR:
                enum ofn = "idfxsr";
                mixin(ofn~"();");
                break;
            case IDSSE:
                enum ofn = "idsse";
                mixin(ofn~"();");
                break;
            case IDSSE2:
                enum ofn = "idsse2";
                mixin(ofn~"();");
                break;
            case IDSS:
                enum ofn = "idss";
                mixin(ofn~"();");
                break;
            case IDHTT:
                enum ofn = "idhtt";
                mixin(ofn~"();");
                break;
            case IDTM:
                enum ofn = "idtm";
                mixin(ofn~"();");
                break;
            case IDIA64:
                enum ofn = "idia64";
                mixin(ofn~"();");
                break;
            case IDPBE:
                enum ofn = "idpbe";
                mixin(ofn~"();");
                break;
            case PFADD:
                enum ofn = "pfadd";
                break;
            case PFSUB:
                enum ofn = "pfsub";
                break;
            case PFSUBR:
                enum ofn = "pfsubr";
                break;
            case PFMUL:
                enum ofn = "pfmul";
                break;
            case PFCMPEQ:
                enum ofn = "pfcmpeq";
                break;
            case PFCMPGE:
                enum ofn = "pfcmpge";
                break;
            case PFCMPGT:
                enum ofn = "pfcmpgt";
                break;
            case PF2ID:
                enum ofn = "pf2id";
                break;
            case PI2FD:
                enum ofn = "pi2fd";
                break;
            case PF2IW:
                enum ofn = "pf2iw";
                break;
            case PI2FW:
                enum ofn = "pi2fw";
                break;
            case PFMAX:
                enum ofn = "pfmax";
                break;
            case PFMIN:
                enum ofn = "pfmin";
                break;
            case PFRCP:
                enum ofn = "pfrcp";
                break;
            case PFRSQRT:
                enum ofn = "pfrsqrt";
                break;
            case PFRCPIT1:
                enum ofn = "pfrcpit1";
                break;
            case PFRSQIT1:
                enum ofn = "pfrsqit1";
                break;
            case PFRCPIT2:
                enum ofn = "pfrcpit2";
                break;
            case PFACC:
                enum ofn = "pfacc";
                break;
            case PFNACC:
                enum ofn = "pfnacc";
                break;
            case PFPNACC:
                enum ofn = "pfpnacc";
                break;
            case PMULHRW:
                enum ofn = "pmulhrw";
                break;
            case PAVGUSB:
                enum ofn = "pavgusb";
                break;
            case PSWAPD:
                enum ofn = "pswapd";
                break;
            case FEMMS:
                enum ofn = "femms";
                mixin(ofn~"();");
                break;
            case ICEBP:
                enum ofn = "icebp";
                mixin(ofn~"();");
                break;
            case PTWRITE:
                enum ofn = "ptwrite";
                break;
            case CLWB:
                enum ofn = "clwb";
                break;
            case CLFLUSHOPT:
                enum ofn = "clflushopt";
                break;
            case STAC:
                enum ofn = "stac";
                mixin(ofn~"();");
                break;
            case CLAC:
                enum ofn = "clac";
                mixin(ofn~"();");
                break;
            case ADC:
                enum ofn = "adc";
                break;
            case ADCX:
                enum ofn = "adcx";
                break;
            case ADOX:
                enum ofn = "adox";
                break;
            case RDSEED:
                enum ofn = "rdseed";
                break;
            case BNDCL:
                enum ofn = "bndcl";
                break;
            case BNDCU:
                enum ofn = "bndcu";
                break;
            case BNDLDX:
                enum ofn = "bndldx";
                break;
            case BNDSTX:
                enum ofn = "bndstx";
                break;
            case BNDMK:
                enum ofn = "bndmk";
                break;
            case BNDMOV:
                enum ofn = "bndmov";
                break;
            case BOUND:
                enum ofn = "bound";
                break;
            case XEND:
                enum ofn = "xend";
                mixin(ofn~"();");
                break;
            case XABORT:
                enum ofn = "xabort";
                break;
            case XBEGIN:
                enum ofn = "xbegin";
                break;
            case XTEST:
                enum ofn = "xtest";
                mixin(ofn~"();");
                break;
            case INVPCID:
                enum ofn = "invpcid";
                break;
            case XACQUIRE:
                enum ofn = "xacquire";
                mixin(ofn~"(0);");
                break;
            case XRELEASE:
                enum ofn = "xrelease";
                mixin(ofn~"(0);");
                break;
            case TZCNT:
                enum ofn = "tzcnt";
                break;
            case LZCNT:
                enum ofn = "lzcnt";
                break;
            case ANDN:
                enum ofn = "andn";
                break;
            case ECREATE:
                enum ofn = "encls_ecreate";
                mixin(ofn~"();");
                break;
            case EINIT:
                enum ofn = "encls_einit";
                mixin(ofn~"();");
                break;
            case EREMOVE:
                enum ofn = "encls_eremove";
                mixin(ofn~"();");
                break;
            case EDBGRD:
                enum ofn = "encls_edbgrd";
                mixin(ofn~"();");
                break;
            case EDBGWR:
                enum ofn = "encls_edbgwr";
                mixin(ofn~"();");
                break;
            case EEXTEND:
                enum ofn = "encls_eextend";
                mixin(ofn~"();");
                break;
            case ELDB:
                enum ofn = "encls_eldb";
                mixin(ofn~"();");
                break;
            case ELDU:
                enum ofn = "encls_eldu";
                mixin(ofn~"();");
                break;
            case EBLOCK:
                enum ofn = "encls_eblock";
                mixin(ofn~"();");
                break;
            case EPA:
                enum ofn = "encls_epa";
                mixin(ofn~"();");
                break;
            case EWB:
                enum ofn = "encls_ewb";
                mixin(ofn~"();");
                break;
            case ETRACK:
                enum ofn = "encls_etrack";
                mixin(ofn~"();");
                break;
            case EAUG:
                enum ofn = "encls_eaug";
                mixin(ofn~"();");
                break;
            case EMODPR:
                enum ofn = "encls_emodpr";
                mixin(ofn~"();");
                break;
            case EMODT:
                enum ofn = "encls_emodt";
                mixin(ofn~"();");
                break;
            case ERDINFO:
                enum ofn = "encls_erdinfo";
                mixin(ofn~"();");
                break;
            case ETRACKC:
                enum ofn = "encls_etrackc";
                mixin(ofn~"();");
                break;
            case ELDBC:
                enum ofn = "encls_eldbc";
                mixin(ofn~"();");
                break;
            case ELDUC:
                enum ofn = "encls_elduc";
                mixin(ofn~"();");
                break;
            case EREPORT:
                enum ofn = "enclu_ereport";
                mixin(ofn~"();");
                break;
            case EGETKEY:
                enum ofn = "enclu_egetkey";
                mixin(ofn~"();");
                break;
            case EENTER:
                enum ofn = "enclu_eenter";
                mixin(ofn~"();");
                break;
            case EEXIT:
                enum ofn = "enclu_eexit";
                mixin(ofn~"();");
                break;
            case EACCEPT:
                enum ofn = "enclu_eaccept";
                mixin(ofn~"();");
                break;
            case EMODPE:
                enum ofn = "enclu_emodpe";
                mixin(ofn~"();");
                break;
            case EACCEPTCOPY:
                enum ofn = "enclu_eacceptcopy";
                mixin(ofn~"();");
                break;
            case EDECCSSA:
                enum ofn = "enclu_edeccssa";
                mixin(ofn~"();");
                break;
            case EDECVIRTCHILD:
                enum ofn = "enclv_edecvirtchild";
                mixin(ofn~"();");
                break;
            case EINCVIRTCHILD:
                enum ofn = "enclv_eincvirtchild";
                mixin(ofn~"();");
                break;
            case ESETCONTEXT:
                enum ofn = "enclv_esetcontext";
                mixin(ofn~"();");
                break;
            case MONITOR:
                enum ofn = "monitor";
                mixin(ofn~"();");
                break;
            case MWAIT:
                enum ofn = "mwait";
                mixin(ofn~"();");
                break;
            case INVVPID:
                enum ofn = "invvpid";
                break;
            case INVEPT:
                enum ofn = "invept";
                break;
            case VMCALL:
                enum ofn = "vmcall";
                mixin(ofn~"();");
                break;
            case VMFUNC:
                enum ofn = "vmfunc";
                mixin(ofn~"();");
                break;
            case VMCLEAR:
                enum ofn = "vmclear";
                break;
            case VMLAUNCH:
                enum ofn = "vmlaunch";
                mixin(ofn~"();");
                break;
            case VMRESUME:
                enum ofn = "vmresume";
                mixin(ofn~"();");
                break;
            case VMXOFF:
                enum ofn = "vmxoff";
                mixin(ofn~"();");
                break;
            case VMXON:
                enum ofn = "vmxon";
                break;
            case VMWRITE:
                enum ofn = "vmwrite";
                break;
            case VMREAD:
                enum ofn = "vmread";
                break;
            case VMPTRST:
                enum ofn = "vmptrst";
                break;
            case VMPTRLD:
                enum ofn = "vmptrld";
                break;
            case CAPABILITIES:
                enum ofn = "getsec_capabilities";
                mixin(ofn~"();");
                break;
            case ENTERACCS:
                enum ofn = "getsec_enteraccs";
                mixin(ofn~"();");
                break;
            case EXITAC:
                enum ofn = "getsec_exitac";
                mixin(ofn~"();");
                break;
            case SENTER:
                enum ofn = "getsec_senter";
                mixin(ofn~"();");
                break;
            case SEXIT:
                enum ofn = "getsec_sexit";
                mixin(ofn~"();");
                break;
            case PARAMETERS:
                enum ofn = "getsec_parameters";
                mixin(ofn~"();");
                break;
            case SMCTRL:
                enum ofn = "getsec_smctrl";
                mixin(ofn~"();");
                break;
            case WAKEUP:
                enum ofn = "getsec_wakeup";
                mixin(ofn~"();");
                break;
            case CMPXCHG16B:
                enum ofn = "cmpxchg16b";
                break;
            case POPCNT:
                enum ofn = "popcnt";
                break;
            case XGETBV:
                enum ofn = "xgetbv";
                mixin(ofn~"();");
                break;
            case XSETBV:
                enum ofn = "xsetbv";
                mixin(ofn~"();");
                break;
            case XRSTOR:
                enum ofn = "xrstor";
                break;
            case XSAVE:
                enum ofn = "xsave";
                break;
            case XRSTORS:
                enum ofn = "xrstors";
                break;
            case XSAVES:
                enum ofn = "xsaves";
                break;
            case XSAVEOPT:
                enum ofn = "xsaveopt";
                break;
            case XSAVEC:
                enum ofn = "xsavec";
                break;
            case RDRAND:
                enum ofn = "rdrand";
                break;
            case FABS:
                enum ofn = "fabs";
                mixin(ofn~"();");
                break;
            case FCHS:
                enum ofn = "fchs";
                mixin(ofn~"();");
                break;
            case FCLEX:
                enum ofn = "fclex";
                mixin(ofn~"();");
                break;
            case FNCLEX:
                enum ofn = "fnclex";
                mixin(ofn~"();");
                break;
            case FADD:
                enum ofn = "fadd";
                break;
            case FADDP:
                enum ofn = "faddp";
                break;
            case FIADD:
                enum ofn = "fiadd";
                break;
            case FBLD:
                enum ofn = "fbld";
                break;
            case FBSTP:
                enum ofn = "fbstp";
                break;
            case FCOM:
                enum ofn = "fcom";
                break;
            case FCOMP:
                enum ofn = "fcomp";
                break;
            case FCOMPP:
                enum ofn = "fcompp";
                break;
            case FCOMI:
                enum ofn = "fcomi";
                break;
            case FCOMIP:
                enum ofn = "fcomip";
                break;
            case FUCOMI:
                enum ofn = "fucomi";
                break;
            case FUCOMIP:
                enum ofn = "fucomip";
                break;
            case FICOM:
                enum ofn = "ficom";
                break;
            case FICOMP:
                enum ofn = "ficomp";
                break;
            case FUCOM:
                enum ofn = "fucom";
                break;
            case FUCOMP:
                enum ofn = "fucomp";
                break;
            case FUCOMPP:
                enum ofn = "fucompp";
                break;
            case FTST:
                enum ofn = "ftst";
                mixin(ofn~"();");
                break;
            case F2XM1:
                enum ofn = "f2xm1";
                mixin(ofn~"();");
                break;
            case FYL2X:
                enum ofn = "fyl2x";
                mixin(ofn~"();");
                break;
            case FYL2XP1:
                enum ofn = "fyl2xp1";
                mixin(ofn~"();");
                break;
            case FCOS:
                enum ofn = "fcos";
                mixin(ofn~"();");
                break;
            case FSIN:
                enum ofn = "fsin";
                mixin(ofn~"();");
                break;
            case FSINCOS:
                enum ofn = "fsincos";
                mixin(ofn~"();");
                break;
            case FSQRT:
                enum ofn = "fsqrt";
                mixin(ofn~"();");
                break;
            case FPTAN:
                enum ofn = "fptan";
                mixin(ofn~"();");
                break;
            case FPATAN:
                enum ofn = "fpatan";
                mixin(ofn~"();");
                break;
            case FPREM:
                enum ofn = "fprem";
                mixin(ofn~"();");
                break;
            case FPREM1:
                enum ofn = "fprem1";
                mixin(ofn~"();");
                break;
            case FDECSTP:
                enum ofn = "fdecstp";
                mixin(ofn~"();");
                break;
            case FINCSTP:
                enum ofn = "fincstp";
                mixin(ofn~"();");
                break;
            case FILD:
                enum ofn = "fild";
                break;
            case FIST:
                enum ofn = "fist";
                break;
            case FISTP:
                enum ofn = "fistp";
                break;
            case FISTTP:
                enum ofn = "fisttp";
                break;
            case FLDCW:
                enum ofn = "fldcw";
                break;
            case FSTCW:
                enum ofn = "fstcw";
                break;
            case FNSTCW:
                enum ofn = "fnstcw";
                break;
            case FLDENV:
                enum ofn = "fldenv";
                break;
            case FSTENV:
                enum ofn = "fstenv";
                break;
            case FNSTENV:
                enum ofn = "fnstenv";
                break;
            case FSTSW:
                enum ofn = "fstsw";
                break;
            case FNSTSW:
                enum ofn = "fnstsw";
                break;
            case FLD:
                enum ofn = "fld";
                break;
            case FLD1:
                enum ofn = "fld1";
                mixin(ofn~"();");
                break;
            case FLDL2T:
                enum ofn = "fldl2t";
                mixin(ofn~"();");
                break;
            case FLDL2E:
                enum ofn = "fldl2e";
                mixin(ofn~"();");
                break;
            case FLDPI:
                enum ofn = "fldpi";
                mixin(ofn~"();");
                break;
            case FLDLG2:
                enum ofn = "fldlg2";
                mixin(ofn~"();");
                break;
            case FLDLN2:
                enum ofn = "fldln2";
                mixin(ofn~"();");
                break;
            case FLDZ:
                enum ofn = "fldz";
                mixin(ofn~"();");
                break;
            case FST:
                enum ofn = "fst";
                break;
            case FSTP:
                enum ofn = "fstp";
                break;
            case FDIV:
                enum ofn = "fdiv";
                break;
            case FDIVP:
                enum ofn = "fdivp";
                break;
            case FIDIV:
                enum ofn = "fidiv";
                break;
            case FDIVR:
                enum ofn = "fdivr";
                break;
            case FDIVRP:
                enum ofn = "fdivrp";
                break;
            case FIDIVR:
                enum ofn = "fidivr";
                break;
            case FSCALE:
                enum ofn = "fscale";
                break;
            case FRNDINT:
                enum ofn = "frndint";
                break;
            case FEXAM:
                enum ofn = "fexam";
                break;
            case FFREE:
                enum ofn = "ffree";
                break;
            case FXCH:
                enum ofn = "fxch";
                break;
            case FXTRACT:
                enum ofn = "fxtract";
                break;
            case FNOP:
                enum ofn = "fnop";
                break;
            case FNINIT:
                enum ofn = "fninit";
                break;
            case FINIT:
                enum ofn = "finit";
                break;
            case FSAVE:
                enum ofn = "fsave";
                break;
            case FNSAVE:
                enum ofn = "fnsave";
                break;
            case FRSTOR:
                enum ofn = "frstor";
                break;
            case FXSAVE:
                enum ofn = "fxsave";
                break;
            case FXRSTOR:
                enum ofn = "fxrstor";
                break;
            case FMUL:
                enum ofn = "fmul";
                break;
            case FMULP:
                enum ofn = "fmulp";
                break;
            case FIMUL:
                enum ofn = "fimul";
                break;
            case FSUB:
                enum ofn = "fsub";
                break;
            case FSUBP:
                enum ofn = "fsubp";
                break;
            case FISUB:
                enum ofn = "fisub";
                break;
            case FSUBR:
                enum ofn = "fsubr";
                break;
            case FSUBRP:
                enum ofn = "fsubrp";
                break;
            case FISUBR:
                enum ofn = "fisubr";
                break;
            case FCMOVCC:
                enum ofn = "fcmovcc";
                break;
            case RDMSR:
                enum ofn = "rdmsr";
                mixin(ofn~"();");
                break;
            case WRMSR:
                enum ofn = "wrmsr";
                mixin(ofn~"();");
                break;
            case CMPXCHG8B:
                enum ofn = "cmpxchg8b";
                break;
            case SYSENTER:
                enum ofn = "sysenter";
                mixin(ofn~"();");
                break;
            case SYSEXITC:
                enum ofn = "sysexitc";
                mixin(ofn~"();");
                break;
            case SYSEXIT:
                enum ofn = "sysexit";
                mixin(ofn~"();");
                break;
            case CMOVCC:
                enum ofn = "cmovcc";
                break;
            case CLFLUSH:
                enum ofn = "clflush";
                break;
            case HRESET:
                enum ofn = "hreset";
                break;
            case INCSSPD:
                enum ofn = "incsspd";
                break;
            case INCSSPQ:
                enum ofn = "incsspq";
                break;
            case CLRSSBSY:
                enum ofn = "clrssbsy";
                break;
            case SETSSBSY:
                enum ofn = "setssbsy";
                mixin(ofn~"();");
                break;
            case RDSSPD:
                enum ofn = "rdsspd";
                break;
            case RDSSPQ:
                enum ofn = "rdsspq";
                break;
            case WRSSD:
                enum ofn = "wrssd";
                break;
            case WRSSQ:
                enum ofn = "wrssq";
                break;
            case WRUSSD:
                enum ofn = "wrussd";
                break;
            case WRUSSQ:
                enum ofn = "wrussq";
                break;
            case RSTORSSP:
                enum ofn = "rstorssp";
                break;
            case SAVEPREVSSP:
                enum ofn = "saveprevssp";
                mixin(ofn~"();");
                break;
            case ENDBR32:
                enum ofn = "endbr32";
                mixin(ofn~"();");
                break;
            case ENDBR64:
                enum ofn = "endbr64";
                mixin(ofn~"();");
                break;
            case RDFSBASE:
                enum ofn = "rdfsbase";
                break;
            case RDGSBASE:
                enum ofn = "rdgsbase";
                break;
            case WRFSBASE:
                enum ofn = "wrfsbase";
                break;
            case WRGSBASE:
                enum ofn = "wrgsbase";
                break;
            case RDPID:
                enum ofn = "rdpid";
                break;
            case WRPKRU:
            // TODO: Pollution for this and MSR?
                enum ofn = "wrpkru";
                mixin(ofn~"();");
                break;
            case RDPKRU:
                enum ofn = "rdpkru";
                mixin(ofn~"();");
                break;
            case TESTUI:
                enum ofn = "testui";
                mixin(ofn~"();");
                break;
            case STUI:
                enum ofn = "stui";
                mixin(ofn~"();");
                break;
            case CLUI:
                enum ofn = "clui";
                mixin(ofn~"();");
                break;
            case UIRET:
                enum ofn = "uiret";
                mixin(ofn~"();");
                break;
            case SENDUIPI:
                enum ofn = "senduipi";
                break;
            case UMWAIT:
                enum ofn = "umwait";
                break;
            case UMONITOR:
                enum ofn = "umonitor";
                break;
            case TPAUSE:
                enum ofn = "tpause";
                break;
            case CLDEMOTE:
                enum ofn = "cldemote";
                break;
            case XRESLDTRK:
                enum ofn = "xresldtrk";
                mixin(ofn~"();");
                break;
            case XSUSLDTRK:
                enum ofn = "xsusldtrk";
                mixin(ofn~"();");
                break;
            case SERIALIZE:
                enum ofn = "serialize";
                mixin(ofn~"();");
                break;
            case PCONFIG:
                enum ofn = "pconfig";
                mixin(ofn~"();");
                break;
            case RDPMC:
                enum ofn = "rdpmc";
                mixin(ofn~"();");
                break;
            case WBINVD:
                enum ofn = "wbinvd";
                mixin(ofn~"();");
                break;
            case WBNOINVD:
                enum ofn = "wbnoinvd";
                mixin(ofn~"();");
                break;
            case INVD:
                enum ofn = "invd";
                mixin(ofn~"();");
                break;
            case LGDT:
                enum ofn = "lgdt";
                break;
            case SGDT:
                enum ofn = "sgdt";
                break;
            case LLDT:
                enum ofn = "lldt";
                break;
            case SLDT:
                enum ofn = "sldt";
                break;
            case LIDT:
                enum ofn = "lidt";
                break;
            case SIDT:
                enum ofn = "sidt";
                break;
            case LMSW:
                enum ofn = "lmsw";
                break;
            case SMSW:
                enum ofn = "smsw";
                break;
            case INVLPG:
                enum ofn = "invlpg";
                break;
            case SAHF:
                enum ofn = "sahf";
                mixin(ofn~"();");
                break;
            case LAHF:
                enum ofn = "lahf";
                mixin(ofn~"();");
                break;
            case SARX:
                enum ofn = "sarx";
                break;
            case SHLX:
                enum ofn = "shlx";
                break;
            case SHRX:
                enum ofn = "shrx";
                break;
            case MOVQ:
                enum ofn = "movq";
                break;
            case MOVD:
                enum ofn = "movd";
                break;
            case ADDPD:
                enum ofn = "addpd";
                break;
            case ADDPS:
                enum ofn = "addps";
                break;
            case ADDSS:
                enum ofn = "addss";
                break;
            case ADDSD:
                enum ofn = "addsd";
                break;
            case LFENCE:
                enum ofn = "lfence";
                mixin(ofn~"();");
                break;
            case SFENCE:
                enum ofn = "sfence";
                mixin(ofn~"();");
                break;
            case MFENCE:
                enum ofn = "mfence";
                mixin(ofn~"();");
                break;
            case ADDSUBPS:
                enum ofn = "addsubps";
                break;
            case ADDSUBPD:
                enum ofn = "addsubpd";
                break;
            case VADDPD:
                enum ofn = "vaddpd";
                break;
            case VADDPS:
                enum ofn = "vaddps";
                break;
            case VADDSD:
                enum ofn = "vaddsd";
                break;
            case VADDSS:
                enum ofn = "vaddss";
                break;
            case VADDSUBPD:
                enum ofn = "vaddsubpd";
                break;
            case VADDSUBPS:
                enum ofn = "vaddsubps";
                break;
            case VMOVQ:
                enum ofn = "vmovq";
                break;
            case VMOVD:
                enum ofn = "vmovd";
                break;
            case AESDEC:
                enum ofn = "aesdec";
                break;
            case VAESDEC:
                enum ofn = "vaesdec";
                break;
            case AESDEC128KL:
                enum ofn = "aesdec128kl";
                break;
            case AESDEC256KL:
                enum ofn = "aesdec256kl";
                break;
            case AESDECLAST:
                enum ofn = "aesdeclast";
                break;
            case VAESDECLAST:
                enum ofn = "vaesdeclast";
                break;
            case AESDECWIDE128KL:
                enum ofn = "aesdecwide128kl";
                break;
            case AESDECWIDE256KL:
                enum ofn = "aesdecwide256kl";
                break;
            case AESENC:
                enum ofn = "aesenc";
                break;
            case VAESENC:
                enum ofn = "vaesenc";
                break;
            case AESENC128KL:
                enum ofn = "aesenc128kl";
                break;
            case AESENC256KL:
                enum ofn = "aesenc256kl";
                break;
            case AESENCLAST:
                enum ofn = "aesenclast";
                break;
            case VAESENCLAST:
                enum ofn = "vaesenclast";
                break;
            case AESENCWIDE128KL:
                enum ofn = "aesencwide128kl";
                break;
            case AESENCWIDE256KL:
                enum ofn = "aesencwide256kl";
                break;
            case AESIMC:
                enum ofn = "aesimc";
                break;
            case VAESIMC:
                enum ofn = "vaesimc";
                break;
            case AESKEYGENASSIST:
                enum ofn = "aeskeygenassist";
                break;
            case VAESKEYGENASSIST:
                enum ofn = "vaeskeygenassist";
                break;
            case SHA1MSG1:
                enum ofn = "sha1msg1";
                break;
            case SHA1MSG2:
                enum ofn = "sha1msg2";
                break;
            case SHA1NEXTE:
                enum ofn = "sha1nexte";
                break;
            case SHA256MSG1:
                enum ofn = "sha256msg1";
                break; 
            case SHA1RNDS4:
                enum ofn = "sha1rnds4";
                break;
            case SHA256RNDS2:
                enum ofn = "sha256rnds2";
                break;
            // TODO: Branch not taken and taken?
            case CRC32:
                enum ofn = "crc32";
                break;
            case ENDQCMD:
                enum ofn = "endqcmd";
                break;
            case CMPXCHG:
                enum ofn = "cmpxchg";
                break;
            case AAA:
                enum ofn = "aaa";
                mixin(ofn~"();");
                break;
            case AAD:
                enum ofn = "aad";
                break;
            case AAM:
                enum ofn = "aam";
                break;
            case AAS:
                enum ofn = "aas";
                mixin(ofn~"();");
                break;
            case ADD:
                enum ofn = "add";
                break;
            case AND:
                enum ofn = "and";
                break;
            case ARPL:
                enum ofn = "arpl";
                break;
            case BSF:
                enum ofn = "bsf";
                break;
            case BSR:
                enum ofn = "bsr";
                break;
            case BSWAP:
                enum ofn = "bswap";
                break;
            case BT:
                enum ofn = "bt";
                break;
            case BTC:
                enum ofn = "btc";
                break;
            case BTR:
                enum ofn = "btr";
                break;
            case BTS:
                enum ofn = "bts";
                break;
            case CMP:
                enum ofn = "cmp";
                break;
            case CWD:
                enum ofn = "cwd";
                mixin(ofn~"();");
                break;
            case CDQ:
                enum ofn = "cdq";
                mixin(ofn~"();");
                break;
            case CQO:
                enum ofn = "cqo";
                mixin(ofn~"();");
                break;
            case CBW:
                enum ofn = "cbw";
                mixin(ofn~"();");
                break;
            case CWDE:
                enum ofn = "cwde";
                mixin(ofn~"();");
                break;
            case CDQE:
                enum ofn = "cdqe";
                mixin(ofn~"();");
                break;
            case CPUID:
                enum ofn = "cpuid";
                break;
            case CLC:
                enum ofn = "clc";
                mixin(ofn~"();");
                break;
            case CLD:
                enum ofn = "cld";
                mixin(ofn~"();");
                break;
            case CLI:
                enum ofn = "cli";
                mixin(ofn~"();");
                break;
            case CLTS:
                enum ofn = "clts";
                mixin(ofn~"();");
                break;
            case CMC:
                enum ofn = "cmc";
                mixin(ofn~"();");
                break;
            case DEC:
                enum ofn = "dec";
                break;
            case INT:
                assert(instr.markFormat("l"));

                if (instr.firstMark(0).b == 3)
                    int3();
                else if (instr.firstMark(0).b == 1)
                    int1();
                else
                    _int(instr.firstMark(0).b);

                break;
            case INTO:
                enum ofn = "into";
                mixin(ofn~"();");
                break;
            case UD:
                assert(instr.markFormat("l") || instr.markFormat("lrn"));
                assert(instr.firstMark(0).d <= 3 && (instr.firstMark(0).d == 2 || instr.operands.length == 3));
                
                if (instr.operands.length == 1)
                    ud2();
                else if (instr.firstMark(0).d == 0)
                {
                    assert(instr.firstMark(1).size == 4 && instr.firstMark(2).size == 4);

                    if (instr.firstMark(2).kind == MarkerKind.REGISTER)
                        ud0(instr.firstMark(1).as!(Reg!32), instr.firstMark(2).as!(Reg!32));
                    else
                        ud0(instr.firstMark(1).as!(Reg!32), instr.firstMark(2).as!(Address!32));
                }
                else if (instr.firstMark(0).d == 1)
                {
                    assert(instr.firstMark(1).size == 4 && instr.firstMark(2).size == 4);

                    if (instr.firstMark(2).kind == MarkerKind.REGISTER)
                        ud1(instr.firstMark(1).as!(Reg!32), instr.firstMark(2).as!(Reg!32));
                    else
                        ud1(instr.firstMark(1).as!(Reg!32), instr.firstMark(2).as!(Address!32));
                }

                break;
            case IRET:
                enum ofn = "iret";
                mixin(ofn~"();");
                break;
            case INC:
                enum ofn = "inc";
                break;
            case HLT:
                enum ofn = "hlt";
                break;
            case PAUSE:
                enum ofn = "pause";
                break;
            case SWAPGS:
                enum ofn = "swapgs";
                break;
            case LOCK:
                enum ofn = "lock";
                mixin(ofn~"(0);");
                break;
            case WAIT:
                enum ofn = "wait";
                mixin(ofn~"();");
                break;
            case FWAIT:
                enum ofn = "fwait";
                mixin(ofn~"();");
                break;
            case SYSRETC:
                enum ofn = "sysretc";
                mixin(ofn~"();");
                break;
            case SYSRET:
                enum ofn = "sysret";
                mixin(ofn~"();");
                break;
            case SYSCALL:
                enum ofn = "syscall";
                mixin(ofn~"();");
                break;
            case RSM:
                enum ofn = "rsm";
                mixin(ofn~"();");
                break;
            case LEAVE:
                enum ofn = "leave";
                mixin(ofn~"();");
                break;
            case ENTER:
                enum ofn = "enter";
                break;
            case LEA:
                enum ofn = "lea";
                break;
            case LDS:
                enum ofn = "lds";
                break;
            case LSS:
                enum ofn = "lss";
                break;
            case LES:
                enum ofn = "les";
                break;
            case LFS:
                enum ofn = "lfs";
                break;
            case LGS:
                enum ofn = "lgs";
                break;
            case LSL:
                enum ofn = "lsl";
                break;
            case LTR:
                enum ofn = "ltr";
                break;
            case STR:
                enum ofn = "str";
                break;
            case NEG:
                enum ofn = "neg";
                break;
            case NOP:
                enum ofn = "nop";
                mixin(ofn~"();");
                break;
            case NOT:
                enum ofn = "not";
                break;
            case RET:
                if (instr.operands.length == 0)
                    ret();
                else
                    ret(instr.firstMark(0).w);

                break;
            case RETF:
                if (instr.operands.length == 0)
                    retf();
                else
                    retf(instr.firstMark(0).w);

                break;
            case STC:
                enum ofn = "stc";
                mixin(ofn~"();");
                break;
            case STD:
                enum ofn = "std";
                mixin(ofn~"();");
                break;
            case STI:
                enum ofn = "sti";
                mixin(ofn~"();");
                break;
            case SUB:
                enum ofn = "sub";
                break;
            case SBB:
                enum ofn = "sbb";
                break;
            case XOR:
                enum ofn = "xor";
                break;
            case OR:
                enum ofn = "or";
                break;
            case SAL:
                enum ofn = "sal";
                break;
            case SAR:
                enum ofn = "sar";
                break;
            case SHL:
                enum ofn = "shl";
                break;
            case SHR:
                enum ofn = "shr";
                break;
            case RCL:
                enum ofn = "rcl";
                break;
            case RCR:
                enum ofn = "rcr";
                break;
            case ROL:
                enum ofn = "rol";
                break;
            case ROR:
                enum ofn = "ror";
                break;
            case VERR:
                enum ofn = "verr";
                break;
            case VERW:
                enum ofn = "verw";
                break;
            case TEST:
                enum ofn = "test";
                break;
            case POP:
                enum ofn = "pop";
                break;
            case POPDS:
                enum ofn = "popds";
                mixin(ofn~"();");
                break;
            case POPES:
                enum ofn = "popes";
                mixin(ofn~"();");
                break;
            case POPSS:
                enum ofn = "popss";
                mixin(ofn~"();");
                break;
            case POPFS:
                enum ofn = "popfs";
                mixin(ofn~"();");
                break;
            case POPGS:
                enum ofn = "popgs";
                mixin(ofn~"();");
                break;
            case POPA:
                enum ofn = "popa";
                mixin(ofn~"();");
                break;
            case POPF:
                enum ofn = "popf";
                mixin(ofn~"();");
                break;
            case PUSH:
                enum ofn = "push";
                break;
            case PUSHCS:
                enum ofn = "pushcs";
                mixin(ofn~"();");
                break;
            case PUSHSS:
                enum ofn = "pushss";
                mixin(ofn~"();");
                break;
            case PUSHDS:
                enum ofn = "pushds";
                mixin(ofn~"();");
                break;
            case PUSHES:
                enum ofn = "pushes";
                mixin(ofn~"();");
                break;
            case PUSHFS:
                enum ofn = "pushfs";
                mixin(ofn~"();");
                break;
            case PUSHGS:
                enum ofn = "pushgs";
                mixin(ofn~"();");
                break;  
            case PUSHA:
                enum ofn = "pusha";
                mixin(ofn~"();");
                break;
            case PUSHF:
                enum ofn = "pushf";
                mixin(ofn~"();");
                break;
            case XADD:
                enum ofn = "xadd";
                break;
            case XCHG:
                enum ofn = "xchg";
                break;
            case XLAT:
                enum ofn = "xlat";
                mixin(ofn~"();");
                break;
            case XLATB:
                enum ofn = "xlatb";
                mixin(ofn~"();");
                break;
            case LAR:
                enum ofn = "lar";
                break;
            case DAA:
                enum ofn = "daa";
                mixin(ofn~"();");
                break;
            case DAS:
                enum ofn = "das";
                mixin(ofn~"();");
                break;
            case MUL:
                enum ofn = "mul";
                break;
            case IMUL:
                enum ofn = "imul";
                break;
            case DIV:
                enum ofn = "div";
                break;
            case IDIV:
                enum ofn = "idiv";
                break;
            case MOV:
                enum ofn = "mov";
                break;
            case MOVSX:
                enum ofn = "movsx";
                break;
            case MOVSXD:
                enum ofn = "movsxd";
                break;
            case MOVZX:
                enum ofn = "movzx";
                break;
            case MOVS:
                enum ofn = "movs";
                break;
            case MOVSB:
                enum ofn = "movsb";
                mixin(ofn~"();");
                break;
            case MOVSW:
                enum ofn = "movsw";
                mixin(ofn~"();");
                break;
            case MOVSD:
                enum ofn = "movsd";
                mixin(ofn~"();");
                break;
            case MOVSQ:
                enum ofn = "movsq";
                mixin(ofn~"();");
                break;
            case CALL:
                enum ofn = "call";
                break;
            case LOOPCC:
                enum ofn = "loop";
                break;
            case JMP:
                enum ofn = "jmp";
                break;
            case JCC:
                enum ofn = "jcc";
                break;
            case REPCC:
                enum ofn = "repcc";
                break;
            case CMPS:
                enum ofn = "cmps";
                break;
            case CMPSB:
                enum ofn = "cmpsb";
                mixin(ofn~"();");
                break;
            case CMPSW:
                enum ofn = "cmpsw";
                mixin(ofn~"();");
                break;
            case CMPSD:
                enum ofn = "cmpsd";
                mixin(ofn~"();");
                break;
            case CMPSQ:
                enum ofn = "cmpsq";
                mixin(ofn~"();");
                break;
            case SCAS:
                enum ofn = "scas";
                break;
            case SCASB:
                enum ofn = "scasb";
                mixin(ofn~"();");
                break;
            case SCASW:
                enum ofn = "scasw";
                mixin(ofn~"();");
                break;
            case SCASD:
                enum ofn = "scasd";
                mixin(ofn~"();");
                break;
            case SCASQ:
                enum ofn = "scasq";
                mixin(ofn~"();");
                break;
            case LODS:
                enum ofn = "lods";
                break;
            case LODSB:
                enum ofn = "lodsb";
                mixin(ofn~"();");
                break;
            case LODSW:
                enum ofn = "lodsw";
                mixin(ofn~"();");
                break;
            case LODSD:
                enum ofn = "lodsd";  
                mixin(ofn~"();");
                break;
            case LODSQ:
                enum ofn = "lodsq";
                mixin(ofn~"();");
                break;
            case STOS:
                enum ofn = "stos";
                break;
            case STOSB:
                enum ofn = "stosb";
                mixin(ofn~"();");
                break;
            case STOSW:
                enum ofn = "stosw";
                mixin(ofn~"();");
                break;
            case STOSD:
                enum ofn = "stosd";
                mixin(ofn~"();");
                break;
            case STOSQ:
                enum ofn = "stosq";
                mixin(ofn~"();");
                break;
            case IN:
                enum ofn = "_in";
                break;
            case INS:
                enum ofn = "ins";
                break;
            case INSB:
                enum ofn = "insb";
                mixin(ofn~"();");
                break;
            case INSW:
                enum ofn = "insw";
                mixin(ofn~"();");
                break;
            case INSD:
                enum ofn = "insd";
                mixin(ofn~"();");
                break;
            case OUT:
                enum ofn = "_out";
                break; 
            case OUTS:
                enum ofn = "outs";
                break;
            case OUTSB:
                enum ofn = "outsb";
                mixin(ofn~"();");
                break;
            case OUTSW:
                enum ofn = "outsw";
                mixin(ofn~"();");
                break;
            case OUTSD:
                enum ofn = "outsd";
                mixin(ofn~"();");
                break;
            case SETCC:
                enum ofn = "setcc";
                break;
            default:
                assert(0, "Unsupported instruction opcode!");
        }
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

    auto pfadd(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x9e);
    auto pfsub(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x9a);
    auto pfsubr(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xaa);
    auto pfmul(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xb4);

    auto pfcmpeq(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xb0);
    auto pfcmpge(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x90);
    auto pfcmpgt(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xa0);

    auto pf2id(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x1d);
    auto pi2fd(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x0d);
    auto pf2iw(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x1c);
    auto pi2fw(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x0c);

    auto pfmax(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xa4);
    auto pfmin(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x94);

    auto pfrcp(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x96);
    auto pfrsqrt(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x97);
    auto pfrcpit1(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xa6);
    auto pfrsqit1(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xa7);
    auto pfrcpit2(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xb6);

    auto pfacc(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xae);
    auto pfnacc(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x8a);
    auto pfpnacc(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0x8e);
    auto pmulhrw(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xb7);

    auto pavgusb(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xbf);
    auto pswapd(RM)(MMX dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x0f, dst, src, 0xbb);

    auto femms() => emit!0(0x0f, 0x0e);
     
    /* ====== ICEBP ====== */
    // Intel exclusive interrupt instruction.

    auto icebp() => emit!0(0xf1);

    /* ====== PT ====== */

    auto ptwrite(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0xf3, 0x0f, 0xae, dst);
    auto ptwrite(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0xf3, 0x0f, 0xae, dst);

    /* ====== CLWB ====== */
    
    auto clwb(RM)(RM dst) if (valid!(RM, 8)) => emit!6(0x66, 0x0f, 0xae, dst);

    /* ====== CLFLUSHOPT ====== */
    
    auto clflushopt(RM)(RM dst) if (valid!(RM, 8)) => emit!7(0x66, 0x0f, 0xae, dst);

    /* ====== SMAP ====== */

    auto stac() => emit!0(0x0f, 0x01, 0xcb);
    auto clac() => emit!0(0x0f, 0x01, 0xca);

    /* ====== ADX ====== */

    auto adc(ubyte imm8) => emit!0(0x14, imm8);
    auto adc(ushort imm16) => emit!0(0x15, imm16);
    auto adc(uint imm32) => emit!0(0x15, imm32);
    auto adc(ulong imm32) => emit!0(0x15, cast(long)imm32);

    auto adc(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!2(0x80, dst, imm8);
    auto adc(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!2(0x81, dst, imm16);
    auto adc(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!2(0x81, dst, imm32);
    auto adc(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!2(0x81, dst, imm32);
    auto adc(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!2(0x83, dst, imm8);
    auto adc(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!2(0x83, dst, imm8);
    auto adc(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!2(0x83, dst, imm8);

    auto adc(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x10, dst, src);
    auto adc(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x11, dst, src);
    auto adc(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x11, dst, src);
    auto adc(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x11, dst, src);

    auto adc(R8 dst, Address!8 src) => emit!0(0x12, dst, src);
    auto adc(R16 dst, Address!16 src) => emit!0(0x13, dst, src);
    auto adc(R32 dst, Address!32 src) => emit!0(0x13, dst, src);
    auto adc(R64 dst, Address!64 src) => emit!0(0x13, dst, src);

    auto adcx(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0F, 0x38, 0xF6, dst, src);
    auto adcx(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0F, 0x38, 0xF6, dst, src);

    auto adox(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xF3, 0x0F, 0x38, 0xF6, dst, src);
    auto adox(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xF3, 0x0F, 0x38, 0xF6, dst, src);

    /* ====== RDSEED ====== */
    
    auto rdseed(R16 dst) => emit!7(0x0f, 0xc7, dst);
    auto rdseed(R32 dst) => emit!7(0x0f, 0xc7, dst);
    auto rdseed(R64 dst) => emit!7(0x0f, 0xc7, dst);

    /* ====== MPX ====== */

    auto bndcl(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0x1a, dst, src);
    auto bndcl(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0x1a, dst, src);

    auto bndcu(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf2, 0x0f, 0x1a, dst, src);
    auto bndcu(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf2, 0x0f, 0x1a, dst, src);

    auto bndcn(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf2, 0x0f, 0x1b, dst, src);
    auto bndcn(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf2, 0x0f, 0x1b, dst, src);

    auto bndldx(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!(0, NP)(0x0f, 0x1a, dst, src);
    auto bndstx(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!(0, NP)(0x0f, 0x1b, dst, src);

    auto bndmk(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0x1b, dst, src);
    auto bndmk(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0x1b, dst, src);

    auto bndmov(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x1a, dst, src);
    auto bndmov(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x1a, dst, src);
    auto bndmov(Address!32 dst, R32 src) => emit!0(0x0f, 0x1b, dst, src);
    auto bndmov(Address!64 dst, R32 src) => emit!0(0x0f, 0x1b, dst, src);

    auto bound(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x62, dst, src);
    auto bound(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x62, dst, src);

    /* ====== RTM ====== */
    
    auto xend() => emit!0(0x0f, 0x01, 0xd5);
    auto xabort(ubyte imm8) => emit!0(0xc6, 0xf8, imm8);
    auto xbegin(ushort rel16) => emit!0(0xc7, 0xf8, rel16);
    auto xbegin(uint rel32) => emit!0(0xc7, 0xf8, rel32);
    auto xtest() => emit!0(0x0f, 0x01, 0xd6);
    
    /* ====== INVPCID ====== */

    auto invpcid(R32 dst, Address!128 src) => emit!0(0x0f, 0x38, 0x82, dst, src);
    auto invpcid(R64 dst, Address!128 src) => emit!0(0x0f, 0x38, 0x82, dst, src);

    /* ====== HLE ====== */
        
    auto xacquire(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~buffer[(buffer.length - size)..$];
        return size + 1;
    }
        
    auto xacquire_lock(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf2~0xf0~buffer[(buffer.length - size)..$];
        return size + 2;
    }
        
    auto xrelease(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0xf3~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    /* ====== BMI1 ====== */

    auto tzcnt(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0xf3, 0x0f, 0xbc, dst, src);
    auto tzcnt(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0xbc, dst, src);
    auto tzcnt(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0xbc, dst, src);

    auto lzcnt(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0xf3, 0x0f, 0xbd, dst, src);
    auto lzcnt(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0xbd, dst, src);
    auto lzcnt(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0xbd, dst, src);

    auto andn(RM)(R32 dst, R32 src, RM stor) if (valid!(RM, 32)) => emit!(0, VEXI, 128, F38, 0)(0xf2, dst, src, stor);
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

    auto invvpid(R64 dst, Address!128 src) => emit!0(0x66, 0x0f, 0x38, 0x81, dst, src);
    auto invvpid(R32 dst, Address!128 src) => emit!0(0x66, 0x0f, 0x38, 0x81, dst, src);
    auto invept(R64 dst, Address!128 src) => emit!0(0x66, 0x0f, 0x38, 0x80, dst, src);
    auto invept(R32 dst, Address!128 src) => emit!0(0x66, 0x0f, 0x38, 0x80, dst, src);

    auto vmcall() => emit!0(0x0f, 0x01, 0xc1);
    auto vmfunc() => emit!0(0x0f, 0x01, 0xd4);
    auto vmclear(RM)(RM dst) if (valid!(RM, 64)) => emit!6(0x66, 0x0f, 0xc7, dst);
    auto vmlaunch() => emit!0(0x0f, 0x01, 0xc2);
    auto vmresume() => emit!0(0x0f, 0x01, 0xc3);
    auto vmxoff() => emit!0(0x0f, 0x01, 0xc4);
    auto vmxon(RM)(RM dst) if (valid!(RM, 64)) => emit!6(0xf3, 0x0f, 0xc7, dst);
    
    auto vmwrite(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!(0, NP)(0x0f, 0x79, dst, src);
    auto vmwrite(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!(0, NP)(0x0f, 0x79, dst, src);
    auto vmread(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!(0, NP)(0x0f, 0x78, dst, src);
    auto vmread(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!(0, NP)(0x0f, 0x78, dst, src);

    auto vmptrst(RM)(RM dst) if (valid!(RM, 64)) => emit!(7, NP)(0x0f, 0xc7, dst);
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

    auto cmpxchg16b(Address!128 dst) => emit!1(0x48, 0x0f, 0xc7, dst);

    /* ====== POPCNT ====== */
    
    auto popcnt(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0xf3, 0x0f, 0xb8, dst, src);
    auto popcnt(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf3, 0x0f, 0xb8, dst, src);
    auto popcnt(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf3, 0x0f, 0xb8, dst, src);
    
    /* ====== XSAVE ====== */
    
    auto xgetbv() => emit!0(0x0f, 0x01, 0xd0);
    auto xsetbv() => emit!0(0x0f, 0x01, 0xd1);

    auto xrstor(RM)(RM dst) if (isInstanceOf!(Address, RM)) => emit!(5, NP)(0x0f, 0xae, dst);
    auto xsave(RM)(RM dst) if (isInstanceOf!(Address, RM)) => emit!(4, NP)(0x0f, 0xae, dst);

    auto xrstors(RM)(RM dst) if (isInstanceOf!(Address, RM)) => emit!(3, NP)(0x0f, 0xc7, dst);
    auto xsaves(RM)(RM dst) if (isInstanceOf!(Address, RM)) => emit!(5, NP)(0x0f, 0xc7, dst);

    auto xsaveopt(RM)(RM dst) if (isInstanceOf!(Address, RM)) => emit!(6, NP)(0x0f, 0xae, dst);
    auto xsavec(RM)(RM dst) if (isInstanceOf!(Address, RM)) => emit!(4, NP)(0x0f, 0xc7, dst);

    /* ====== RDRAND ====== */

    auto rdrand(R16 dst) => emit!6(0x0f, 0xc7, dst);
    auto rdrand(R32 dst) => emit!6(0x0f, 0xc7, dst);
    auto rdrand(R64 dst) => emit!6(0x0f, 0xc7, dst);

    /* ====== FPU ====== */

    auto fabs() => emit!0(0xd9, 0xe1);
    auto fchs() => emit!0(0xd9, 0xe0);

    auto fclex() => emit!0(0x9b, 0xdb, 0xe2);
    auto fnclex() => emit!0(0xdb, 0xe2);

    auto fadd(Address!32 dst) => emit!(0, NP)(0xd8, dst);
    auto fadd(Address!64 dst) => emit!(0, NP)(0xdc, dst);
    auto fadd(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xc0, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xc0, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto faddp(Reg!(-3) dst) => emit!(0, NRM)(0xde, 0xc0, dst);
    auto fiadd(Address!32 dst) => emit!(0, NP)(0xda, dst);
    auto fiadd(Address!16 dst) => emit!(0, NP)(0xde, dst);

    auto fbld(Address!80 dst) => emit!(4, NP)(0xdf, dst);
    auto fbstp(Address!80 dst) => emit!(6, NP)(0xdf, dst);

    auto fcom(Address!32 dst) => emit!(2, NP)(0xd8, dst);
    auto fcom(Address!64 dst) => emit!(2, NP)(0xdc, dst);
    auto fcom(Reg!(-3) dst) => emit!(2, NRM)(0xd8, 0xd0, dst);

    auto fcomp(Address!32 dst) => emit!(3, NP)(0xd8, dst);
    auto fcomp(Address!64 dst) => emit!(3, NP)(0xdc, dst);
    auto fcomp(Reg!(-3) dst) => emit!(2, NRM)(0xd8, 0xd8, dst);
    auto fcompp() => emit!0(0xde, 0xd9);

    auto fcomi(Reg!(-3) dst) => emit!(0, NRM)(0xdb, 0xf0, dst);
    auto fcomip(Reg!(-3) dst) => emit!(0, NRM)(0xdf, 0xf0, dst);
    auto fucomi(Reg!(-3) dst) => emit!(0, NRM)(0xdb, 0xe8, dst);
    auto fucomip(Reg!(-3) dst) => emit!(0, NRM)(0xdf, 0xe8, dst);

    auto ficom(Address!16 dst) => emit!(2, NP)(0xde, dst);
    auto ficom(Address!32 dst) => emit!(2, NP)(0xda, dst);
    auto ficomp(Address!16 dst) => emit!(2, NP)(0xde, dst);
    auto ficomp(Address!32 dst) => emit!(2, NP)(0xda, dst);
    
    auto fucom(Reg!(-3) dst) => emit!(2, NRM)(0xdd, 0xe0, dst);
    auto fucomp(Reg!(-3) dst) => emit!(2, NRM)(0xdd, 0xe8, dst);
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
    auto fstsw() => emit!0(0x9b, 0xdf, 0xe0);
    auto fnstsw(Address!16 dst) => emit!(7, NP)(0xdd, dst);
    auto fnstsw() => emit!0(0xdf, 0xe0);

    auto fld(Address!32 dst) => emit!(0, NP)(0xd9, dst);
    auto fld(Address!64 dst) => emit!(0, NP)(0xdd, dst);
    auto fld(Address!80 dst) => emit!(5, NP)(0xdb, dst);
    auto fld(Reg!(-3) dst) => emit!(0, NRM)(0xd9, 0xc0, dst);

    auto fld1() => emit!0(0xd9, 0xe8);
    auto fldl2t() => emit!0(0xd9, 0xe9);
    auto fldl2e() => emit!0(0xd9, 0xea);
    auto fldpi() => emit!0(0xd9, 0xeb);
    auto fldlg2() => emit!0(0xd9, 0xec);
    auto fldln2() => emit!0(0xd9, 0xed);
    auto fldz() => emit!0(0xd9, 0xee);

    auto fst(Address!32 dst) => emit!(2, NP)(0xd9, dst);
    auto fst(Address!64 dst) => emit!(2, NP)(0xdd, dst);
    auto fst(Reg!(-3) dst) => emit!(0, NRM)(0xdd, 0xd0, dst);
    
    auto fstp(Address!32 dst) => emit!(3, NP)(0xd9, dst);
    auto fstp(Address!64 dst) => emit!(3, NP)(0xdd, dst);
    auto fstp(Address!80 dst) => emit!(7, NP)(0xdb, dst);
    auto fstp(Reg!(-3) dst) => emit!(0, NRM)(0xdd, 0xd8, dst);

    auto fdiv(Address!32 dst) => emit!(6, NP)(0xd8, dst);
    auto fdiv(Address!64 dst) => emit!(6, NP)(0xdc, dst);
    auto fdiv(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xf0, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xf8, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fdivp(Reg!(-3) dst) => emit!(0, NRM)(0xde, 0xf8, dst);
    auto fidiv(Address!32 dst) => emit!(6, NP)(0xda, dst);
    auto fidiv(Address!16 dst) => emit!(6, NP)(0xde, dst);

    auto fdivr(Address!32 dst) => emit!(7, NP)(0xd8, dst);
    auto fdivr(Address!64 dst) => emit!(7, NP)(0xdc, dst);
    auto fdivr(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xf8, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xf0, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fdivrp(Reg!(-3) dst) => emit!(0, NRM)(0xde, 0xf0, dst);
    auto fidivr(Address!32 dst) => emit!(7, NP)(0xda, dst);
    auto fidivr(Address!16 dst) => emit!(7, NP)(0xde, dst);

    auto fscale() => emit!0(0xd9, 0xfd);
    auto frndint() => emit!0(0xd9, 0xfc);
    auto fexam() => emit!0(0xd9, 0xe5);
    auto ffree(Reg!(-3) dst) => emit!(0, NRM)(0xdd, 0xc0, dst);
    auto fxch(Reg!(-3) dst) => emit!(0, NRM)(0xd9, 0xc8, dst);
    auto fxtract() => emit!0(0xd9, 0xf4);

    auto fnop() => emit!0(0xd9, 0xd0);
    auto fninit() => emit!0(0x9b, 0xdb, 0xe3);
    auto finit() => emit!0(0xdb, 0xe3);

    auto fsave(Address!752 dst) => emit!6(0x9b, 0xdd, dst);
    auto fsave(Address!864 dst) => emit!6(0x9b, 0xdd, dst);
    auto fnsave(Address!752 dst) => emit!6(0xdd, dst);
    auto fnsave(Address!864 dst) => emit!6(0xdd, dst);

    auto frstor(Address!752 dst) => emit!4(0xdd, dst);
    auto frstor(Address!864 dst) => emit!4(0xdd, dst);

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
            emit!(0, NRM)(0xd8, 0xc8, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xc8, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fmulp(Reg!(-3) dst) => emit!(0, NRM)(0xde, 0xc8, dst);
    auto fimul(Address!32 dst) => emit!(1, NP)(0xda, dst);
    auto fimul(Address!16 dst) => emit!(1, NP)(0xde, dst);

    auto fsub(Address!32 dst) => emit!(4, NP)(0xd8, dst);
    auto fsub(Address!64 dst) => emit!(4, NP)(0xdc, dst);
    auto fsub(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xe0, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xe8, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fsubp(Reg!(-3) dst) => emit!(0, NRM)(0xde, 0xe8, dst);
    auto fisub(Address!32 dst) => emit!(4, NP)(0xda, dst);
    auto fisub(Address!16 dst) => emit!(4, NP)(0xde, dst);

    auto fsubr(Address!32 dst) => emit!(5, NP)(0xd8, dst);
    auto fsubr(Address!64 dst) => emit!(5, NP)(0xdc, dst);
    auto fsubr(Reg!(-3) dst, Reg!(-3) src)
    {
        if (dst.index == 0)
            emit!(0, NRM)(0xd8, 0xe8, src);
        else if (src.index == 0)
            emit!(0, NRM)(0xdc, 0xe0, dst);
        else
            assert(0, "Cannot encode 'fadd' with no 'st0' operand!");
    }
    auto fsubrp(Reg!(-3) dst) => emit!(0, NRM)(0xde, 0xe0, dst);
    auto fisubr(Address!32 dst) => emit!(5, NP)(0xda, dst);
    auto fisubr(Address!16 dst) => emit!(5, NP)(0xde, dst);

    auto fcmovb(Reg!(-3) dst) => emit!(0, NRM)(0xda, 0xc0, dst);
    auto fcmove(Reg!(-3) dst) => emit!(0, NRM)(0xda, 0xc8, dst);
    auto fcmovbe(Reg!(-3) dst) => emit!(0, NRM)(0xda, 0xd0, dst);
    auto fcmovu(Reg!(-3) dst) => emit!(0, NRM)(0xda, 0xd8, dst);
    auto fcmovnb(Reg!(-3) dst) => emit!(0, NRM)(0xdb, 0xc0, dst);
    auto fcmovne(Reg!(-3) dst) => emit!(0, NRM)(0xdb, 0xc8, dst);
    auto fcmovnbe(Reg!(-3) dst) => emit!(0, NRM)(0xdb, 0xd0, dst);
    auto fcmovnu(Reg!(-3) dst) => emit!(0, NRM)(0xdb, 0xd8, dst);

    /* ====== TSC ====== */

    // TODO: Not in the new enum?
    auto rdtsc() => emit!0(0x0f, 0x31);
    auto rdtscp() => emit!0(0x0f, 0x01, 0xf9);

    /* ====== MSR ====== */

    auto rdmsr() => emit!0(0x0f, 0x32);
    auto wrmsr() => emit!0(0x0f, 0x30);
    
    /* ====== CX8 ====== */

    auto cmpxchg8b(Address!64 dst) => emit!(1, NP)(0x0f, 0xc7, dst);

    /* ====== SEP ====== */
    
    auto sysenter() => emit!0(0x0f, 0x34);
    auto sysexitc() => emit!0(0x0f, 0x35);
    auto sysexit() => emit!0(0x0f, 0x35);

    /* ====== CMOV ====== */

    auto cmova(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x47, dst, src);
    auto cmova(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x47, dst, src);
    auto cmova(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x47, dst, src);
    
    auto cmovae(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x43, dst, src);
    auto cmovae(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x43, dst, src);
    auto cmovae(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x43, dst, src);
    
    auto cmovb(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x42, dst, src);
    auto cmovb(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x42, dst, src);
    auto cmovb(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x42, dst, src);
    
    auto cmovbe(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x46, dst, src);
    auto cmovbe(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x46, dst, src);
    auto cmovbe(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x46, dst, src);
    
    auto cmovc(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x42, dst, src);
    auto cmovc(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x42, dst, src);
    auto cmovc(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x42, dst, src);
    
    auto cmove(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x44, dst, src);
    auto cmove(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x44, dst, src);
    auto cmove(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x44, dst, src);
    
    auto cmovg(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4f, dst, src);
    auto cmovg(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4f, dst, src);
    auto cmovg(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4f, dst, src);
    
    auto cmovge(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4d, dst, src);
    auto cmovge(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4d, dst, src);
    auto cmovge(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4d, dst, src);
    
    auto cmovl(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4c, dst, src);
    auto cmovl(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4c, dst, src);
    auto cmovl(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4c, dst, src);
    
    auto cmovle(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4e, dst, src);
    auto cmovle(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4e, dst, src);
    auto cmovle(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4e, dst, src);
    
    auto cmovna(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x46, dst, src);
    auto cmovna(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x46, dst, src);
    auto cmovna(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x46, dst, src);
    
    auto cmovnae(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x42, dst, src);
    auto cmovnae(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x42, dst, src);
    auto cmovnae(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x42, dst, src);
    
    auto cmovnb(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x43, dst, src);
    auto cmovnb(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x43, dst, src);
    auto cmovnb(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x43, dst, src);
    
    auto cmovnbe(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x47, dst, src);
    auto cmovnbe(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x47, dst, src);
    auto cmovnbe(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x47, dst, src);
    
    auto cmovnc(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x43, dst, src);
    auto cmovnc(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x43, dst, src);
    auto cmovnc(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x43, dst, src);
    
    auto cmovne(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x45, dst, src);
    auto cmovne(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x45, dst, src);
    auto cmovne(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x45, dst, src);
    
    auto cmovng(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4e, dst, src);
    auto cmovng(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4e, dst, src);
    auto cmovng(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4e, dst, src);
    
    auto cmovnge(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4c, dst, src);
    auto cmovnge(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4c, dst, src);
    auto cmovnge(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4c, dst, src);
    
    auto cmovnl(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4d, dst, src);
    auto cmovnl(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4d, dst, src);
    auto cmovnl(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4d, dst, src);
    
    auto cmovnle(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4f, dst, src);
    auto cmovnle(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4f, dst, src);
    auto cmovnle(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4f, dst, src);
    
    auto cmovno(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x41, dst, src);
    auto cmovno(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x41, dst, src);
    auto cmovno(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x41, dst, src);
    
    auto cmovnp(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4b, dst, src);
    auto cmovnp(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4b, dst, src);
    auto cmovnp(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4b, dst, src);
    
    auto cmovns(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x49, dst, src);
    auto cmovns(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x49, dst, src);
    auto cmovns(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x49, dst, src);
    
    auto cmovnz(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x45, dst, src);
    auto cmovnz(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x45, dst, src);
    auto cmovnz(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x45, dst, src);
    
    auto cmovo(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x40, dst, src);
    auto cmovo(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x40, dst, src);
    auto cmovo(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x40, dst, src);
    
    auto cmovp(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4a, dst, src);
    auto cmovp(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4a, dst, src);
    auto cmovp(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4a, dst, src);
    
    auto cmovpe(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4a, dst, src);
    auto cmovpe(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4a, dst, src);
    auto cmovpe(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4a, dst, src);
    
    auto cmovpo(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x4b, dst, src);
    auto cmovpo(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x4b, dst, src);
    auto cmovpo(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x4b, dst, src);
    
    auto cmovs(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x48, dst, src);
    auto cmovs(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x48, dst, src);
    auto cmovs(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x48, dst, src);
    
    auto cmovz(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x44, dst, src);
    auto cmovz(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0x44, dst, src);
    auto cmovz(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0x44, dst, src);

    /* ====== CLFL ====== */

    auto clflush(RM)(RM dst) if (valid!(RM, 8)) => emit!(7, NP)(0x0f, 0xae, dst);

    /* ====== HRESET ====== */

    auto hreset(ubyte imm8) => emit!0(0xf3, 0x0f, 0x3a, 0xf0, 0xc0, imm8, eax);

    /* ====== CET ====== */
    // Shadow stack instruction set.

    auto incsspd(R32 dst) => emit!5(0xf3, 0x0f, 0xae, dst);
    auto incsspq(R64 dst) => emit!5(0xf3, 0x0f, 0xae, dst);
    auto clrssbsy(Address!64 dst) => emit!6(0xf3, 0x0f, 0xae, dst);
    auto setssbsy() => emit!0(0xf3, 0x0f, 0x01, 0xe8);

    auto rdsspd(R32 dst) => emit!1(0xf3, 0x0f, 0x1e, dst);
    auto rdsspq(R64 dst) => emit!1(0xf3, 0x0f, 0x1e, dst);
    auto wrssd(Address!32 dst, R32 src) => emit!0(0xf3, 0x38, 0xf6, dst, src);
    auto wrssq(Address!64 dst, R64 src) => emit!0(0xf3, 0x38, 0xf6, dst, src);
    auto wrussd(Address!32 dst, R32 src) => emit!1(0x66, 0xf3, 0x38, 0xf5, dst, src);
    auto wrussq(Address!64 dst, R64 src) => emit!1(0x66, 0xf3, 0x38, 0xf5, dst, src);

    auto rstorssp(Address!64 dst) => emit!5(0xf3, 0x0f, 0x01, dst);
    auto saveprevssp() => emit!5(0xf3, 0x0f, 0x01, 0xae, edx);

    auto endbr32() => emit!0(0xf3, 0x0f, 0x1e, 0xfb);
    auto endbr64() => emit!0(0xf3, 0x0f, 0x1e, 0xfa);

    /* ====== FSGSBASE ====== */

    auto rdfsbase(R32 dst) => emit!0(0xf3, 0x0f, 0xae, dst);
    auto rdfsbase(R64 dst) => emit!0(0xf3, 0x0f, 0xae, dst);
    auto rdgsbase(R32 dst) => emit!1(0xf3, 0x0f, 0xae, dst);
    auto rdgsbase(R64 dst) => emit!1(0xf3, 0x0f, 0xae, dst);

    auto wrfsbase(R32 dst) => emit!2(0xf3, 0x0f, 0xae, dst);
    auto wrfsbase(R64 dst) => emit!2(0xf3, 0x0f, 0xae, dst);
    auto wrgsbase(R32 dst) => emit!3(0xf3, 0x0f, 0xae, dst);
    auto wrgsbase(R64 dst) => emit!3(0xf3, 0x0f, 0xae, dst);

    /* ====== RDPID ====== */

    auto rdpid(R32 dst) => emit!7(0xf3, 0x0f, 0xc7, dst);
    auto rdpid(R64 dst) => emit!7(0xf3, 0x0f, 0xc7, dst);

    /* ====== OSPKE ====== */

    auto wrpkru() => emit!0(0x0f, 0x01, 0xef);
    auto rdpkru() => emit!0(0x0f, 0x01, 0xee);

    /* ====== UINTR ====== */

    auto testui() => emit!0(0xf3, 0x0f, 0x01, 0xed);
    auto stui() => emit!0(0xf3, 0x0f, 0x01, 0xef);
    auto clui() => emit!0(0xf3, 0x0f, 0x01, 0xee);
    auto uiret() => emit!0(0xf3, 0x0f, 0x01, 0xec);
    auto senduipi(RM)(RM dst) if (valid!(RM, 8) || valid!(RM, 16) || valid!(RM, 32) || valid!(RM, 64)) => emit!6(0xf3, 0x0f, 0xc7, dst);

    /* ====== WAITPKG ====== */
    
    auto umwait(R32 dst) => emit!6(0xf2, 0x0f, 0xae, dst);
    auto umonitor(R16 dst) => emit!6(0xf3, 0x0f, 0xae, dst);
    auto umonitor(R32 dst) => emit!6(0xf3, 0x0f, 0xae, dst);
    auto umonitor(R64 dst) => emit!6(0xf3, 0x0f, 0xae, dst);
    auto tpause(R32 dst) => emit!6(0x0f, 0xae, dst);

    /* ====== CLDEMOTE ====== */
    
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

    auto lgdt(RM)(RM dst) if (valid!(RM, 32)) => emit!2(0x0f, 0x01, dst);
    auto lgdt(RM)(RM dst) if (valid!(RM, 64)) => emit!2(0x0f, 0x01, dst);
    auto sgdt(RM)(RM dst) if (valid!(RM, 64)) => emit!0(0x0f, 0x01, dst);

    auto lldt(RM)(RM dst) if (valid!(RM, 16)) => emit!2(0x0f, 0x00, dst);
    auto sldt(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0x0f, 0x00, dst);

    auto lidt(RM)(RM dst) if (valid!(RM, 32)) => emit!3(0x0f, 0x01, dst);
    auto lidt(RM)(RM dst) if (valid!(RM, 64)) => emit!3(0x0f, 0x01, dst);
    auto sidt(RM)(RM dst) if (valid!(RM, 64)) => emit!1(0x0f, 0x01, dst);

    auto lmsw(RM)(RM dst) if (valid!(RM, 16)) => emit!6(0x0f, 0x01, dst);

    auto smsw(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0x0f, 0x01, dst);
    auto smsw(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0x0f, 0x01, dst);
    auto smsw(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0x0f, 0x01, dst);

    /* ====== PCID ====== */

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
    auto movq(Address!64 dst, MMX src) => emit!(0, SSE)(0x0f, 0x7f, dst, src);

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
    auto movq(Address!64 dst, XMM src) => emit!(0, SSE)(0x66, 0x0f, 0xd6, dst, src);

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
    auto vmovq(Address!64 dst, XMM src) => emit!(0, VEX, 128, DEFAULT, 0x66)(0xd6, dst, src);

    auto vmovd(RM)(XMM dst, RM src) if (valid!(RM, 32)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x6e, dst, src);
    auto vmovd(RM)(RM dst, XMM src) if (valid!(RM, 32)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x7e, dst, src);

    auto vmovq(RM)(XMM dst, RM src) if (valid!(RM, 64)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x6e, dst, src);
    auto vmovq(RM)(RM dst, XMM src) if (valid!(RM, 64)) => emit!(0, VEX, 128, DEFAULT, 0x66)(0x7e, dst, src);

    /* ====== AES ====== */

    auto aesdec(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xde, dst, src);
    auto vaesdec(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xde, dst, src, stor);
    auto vaesdec(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, F38, 0x66)(0xde, dst, src, stor);

    auto aesdec128kl(XMM dst, Address!384 src) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xdd, dst, src);
    auto aesdec256kl(XMM dst, Address!512 src) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xdf, dst, src);

    auto aesdeclast(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xdf, dst, src);
    auto vaesdeclast(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xdf, dst, src, stor);
    auto vaesdeclast(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, F38, 0x66)(0xdf, dst, src, stor);

    auto aesdecwide128kl(Address!384 dst) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xd8, dst, ecx);
    auto aesdecwide256kl(Address!512 dst) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xd8, dst, ebx);

    auto aesenc(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xdc, dst, src);
    auto vaesenc(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xdc, dst, src, stor);
    auto vaesenc(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, F38, 0x66)(0xdc, dst, src, stor);

    auto aesenc128kl(XMM dst, Address!384 src) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xdc, dst, src);
    auto aesenc256kl(XMM dst, Address!512 src) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xde, dst, src);

    auto aesenclast(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xdd, dst, src);
    auto vaesenclast(RM)(XMM dst, XMM src, RM stor) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xdd, dst, src, stor);
    auto vaesenclast(RM)(YMM dst, YMM src, RM stor) if (valid!(RM, 256)) => emit!(0, VEX, 256, F38, 0x66)(0xdd, dst, src, stor);

    auto aesencwide128kl(Address!384 dst) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xd8, dst, eax);
    auto aesencwide256kl(Address!512 dst) => emit!(0, SSE)(0xf3, 0x0f, 0x38, 0xd8, dst, edx);

    auto aesimc(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x38, 0xdb, dst, src);
    auto vaesimc(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, VEX, 128, F38, 0x66)(0xdb, dst, src);

    auto aeskeygenassist(RM)(XMM dst, RM src, ubyte imm8) if (valid!(RM, 128)) => emit!(0, SSE)(0x66, 0x0f, 0x3a, 0xdf, dst, src, imm8);
    auto vaeskeygenassist(RM)(XMM dst, RM src, ubyte imm8) if (valid!(RM, 128)) => emit!(0, VEX, 128, F3A, 0x66)(0xdf, dst, src, imm8);

    /* ====== SHA ====== */

    auto sha1msg1(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xc9, dst, src);
    auto sha1msg2(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xca, dst, src);
    auto sha1nexte(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xc8, dst, src);

    auto sha256msg1(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xcc, dst, src);

    auto sha1rnds4(RM)(XMM dst, RM src, ubyte imm8) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x3a, 0xcc, dst, src, imm8);

    auto sha256rnds2(RM)(XMM dst, RM src) if (valid!(RM, 128)) => emit!(0, SSE)(0x0f, 0x38, 0xcb, dst, src);

    /* ====== MAIN ====== */

    // NOTE: Branch hints are generally useless in the modern day, AMD CPUs don't even acknowledge them;
    // and thus these should not be used on any modern CPU.

    auto not_taken(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0x2e~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    auto taken(size_t size)
    {
        buffer = buffer[0..(buffer.length - size)]~0x3e~buffer[(buffer.length - size)..$];
        return size + 1;
    }

    auto crc32(RM)(R32 dst, RM src) if (valid!(RM, 8)) => emit!0(0xf2, 0x0f, 0x38, 0xf0, dst, src);
    auto crc32(RM)(R32 dst, RM src) if (valid!(RM, 16)) => emit!0(0xf2, 0x0f, 0x38, 0xf1, dst, src);
    auto crc32(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0xf2, 0x0f, 0x38, 0xf1, dst, src);

    auto crc32(RM)(R64 dst, RM src) if (valid!(RM, 8)) => emit!0(0xf2, 0x0f, 0x38, 0xf0, dst, src);
    auto crc32(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0xf2, 0x0f, 0x38, 0xf1, dst, src);

    // literally 1984
    auto enqcmd(R32 dst, Address!512 src) => emit!0(0xf2, 0x0f, 0x38, 0xf8, dst, src);
    auto enqcmd(R64 dst, Address!512 src) => emit!0(0xf2, 0x0f, 0x38, 0xf8, dst, src);

    auto cmpxchg(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x0f, 0xb0, dst, src);
    auto cmpxchg(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xb1, dst, src);
    auto cmpxchg(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xb1, dst, src);
    auto cmpxchg(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xb1, dst, src);

    auto aaa() => emit!0(0x37);
    auto aad() => emit!0(0xd5, 0x0a);
    auto aad(ubyte imm8) => emit!0(0xd5, imm8);
    auto aam() => emit!0(0xd4, 0x0a);
    auto aam(ubyte imm8) => emit!0(0xd4, imm8);
    auto aas() => emit!0(0x3f);

    auto add(ubyte imm8) => emit!0(0x04, imm8);
    auto add(ushort imm16) => emit!0(0x05, imm16);
    auto add(uint imm32) => emit!0(0x05, imm32);
    auto add(ulong imm32) => emit!0(0x05, cast(long)imm32);

    auto add(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!0(0x80, dst, imm8);
    auto add(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!0(0x81, dst, imm16);
    auto add(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!0(0x81, dst, imm32);
    auto add(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!0(0x81, dst, imm32);
    auto add(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!0(0x83, dst, imm8);
    auto add(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!0(0x83, dst, imm8);
    auto add(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!0(0x83, dst, imm8);

    auto add(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x00, dst, src);
    auto add(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x01, dst, src);
    auto add(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x01, dst, src);
    auto add(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x01, dst, src);

    auto add(R8 dst, Address!8 src) => emit!0(0x02, dst, src);
    auto add(R16 dst, Address!16 src) => emit!0(0x03, dst, src);
    auto add(R32 dst, Address!32 src) => emit!0(0x03, dst, src);
    auto add(R64 dst, Address!64 src) => emit!0(0x03, dst, src);

    auto and(ubyte imm8) => emit!0(0x24, imm8);
    auto and(ushort imm16) => emit!0(0x25, imm16);
    auto and(uint imm32) => emit!0(0x25, imm32);
    auto and(ulong imm32) => emit!0(0x25, cast(long)imm32);

    auto and(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!4(0x80, dst, imm8);
    auto and(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!4(0x81, dst, imm16);
    auto and(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!4(0x81, dst, imm32);
    auto and(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!4(0x81, dst, imm32);
    auto and(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!4(0x83, dst, imm8);
    auto and(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!4(0x83, dst, imm8);
    auto and(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!4(0x83, dst, imm8);

    auto and(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x20, dst, src);
    auto and(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x21, dst, src);
    auto and(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x21, dst, src);
    auto and(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x21, dst, src);

    auto and(R8 dst, Address!8 src) => emit!0(0x22, dst, src);
    auto and(R16 dst, Address!16 src) => emit!0(0x23, dst, src);
    auto and(R32 dst, Address!32 src) => emit!0(0x23, dst, src);
    auto and(R64 dst, Address!64 src) => emit!0(0x23, dst, src);

    auto arpl(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x63, dst, src);

    auto bsf(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbc, dst, src);
    auto bsf(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xbc, dst, src);
    auto bsf(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0xbc, dst, src);

    auto bsr(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbd, dst, src);
    auto bsr(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xbd, dst, src);
    auto bsr(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0xbd, dst, src);

    auto bswap(R32 dst) => emit!(0, NRM)(0x0f, 0xc8, dst);
    auto bswap(R64 dst) => emit!(0, NRM)(0x0f, 0xc8, dst);

    auto bt(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xa3, dst, src); 
    auto bt(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xa3, dst, src); 
    auto bt(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xa3, dst, src); 
    auto bt(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!4(0x0f, 0xba, dst, imm8); 
    auto bt(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!4(0x0f, 0xba, dst, imm8); 
    auto bt(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!4(0x0f, 0xba, dst, imm8); 

    auto btc(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbb, dst, src); 
    auto btc(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xbb, dst, src); 
    auto btc(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xbb, dst, src); 
    auto btc(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!7(0x0f, 0xba, dst, imm8); 
    auto btc(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!7(0x0f, 0xba, dst, imm8); 
    auto btc(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!7(0x0f, 0xba, dst, imm8); 

    auto btr(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xb3, dst, src); 
    auto btr(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xb3, dst, src); 
    auto btr(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xb3, dst, src); 
    auto btr(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!6(0x0f, 0xba, dst, imm8); 
    auto btr(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!6(0x0f, 0xba, dst, imm8); 
    auto btr(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!6(0x0f, 0xba, dst, imm8); 

    auto bts(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xab, dst, src); 
    auto bts(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xab, dst, src); 
    auto bts(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xab, dst, src); 
    auto bts(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!5(0x0f, 0xba, dst, imm8); 
    auto bts(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!5(0x0f, 0xba, dst, imm8); 
    auto bts(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!5(0x0f, 0xba, dst, imm8);

    auto cmp(ubyte imm8) => emit!0(0x3c, imm8);
    auto cmp(ushort imm16) => emit!0(0x3d, imm16);
    auto cmp(uint imm32) => emit!0(0x3d, imm32);
    auto cmp(ulong imm32) => emit!0(0x3d, cast(long)imm32);

    auto cmp(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!7(0x80, dst, imm8);
    auto cmp(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!7(0x81, dst, imm16);
    auto cmp(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!7(0x81, dst, imm32);
    auto cmp(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!7(0x81, dst, imm32);
    auto cmp(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!7(0x83, dst, imm8); 
    auto cmp(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!7(0x83, dst, imm8); 
    auto cmp(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!7(0x83, dst, imm8); 

    auto cmp(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x38, dst, src);
    auto cmp(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x39, dst, src);
    auto cmp(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x39, dst, src);
    auto cmp(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x39, dst, src);

    auto cmp(R8 dst, Address!8 src) => emit!0(0x3a, dst, src);
    auto cmp(R16 dst, Address!16 src) => emit!0(0x3b, dst, src);
    auto cmp(R32 dst, Address!32 src) => emit!0(0x3b, dst, src);
    auto cmp(R64 dst, Address!64 src) => emit!0(0x3b, dst, src);

    auto cwd() => emit!0(0x66, 0x99);
    auto cdq() => emit!0(0x99);
    auto cqo() => emit!0(0x48, 0x99);

    auto cbw() => emit!0(0x66, 0x98);
    auto cwde() => emit!0(0x98);
    auto cdqe() => emit!0(0x48, 0x98);

    auto cpuid() => emit!0(0x0f, 0xa2);
    auto cpuid(uint imm32) => mov(eax, imm32) + cpuid();

    auto clc() => emit!0(0xf8);
    auto cld() => emit!0(0xfc);
    auto cli() => emit!0(0xfa);
    auto clts() => emit!0(0x0f, 0x06);

    auto cmc() => emit!0(0xf5);

    auto dec(RM)(RM dst) if (valid!(RM, 8)) => emit!1(0xfe, dst);
    static if (X64)
    auto dec(RM)(RM dst) if (valid!(RM, 16)) => emit!1(0xff, dst);
    static if (!X64)
    auto dec(Address!16 dst) => emit!1(0xff, dst);
    static if (X64)
    auto dec(RM)(RM dst) if (valid!(RM, 32)) => emit!1(0xff, dst);
    static if (!X64)
    auto dec(Address!32 dst) => emit!1(0xff, dst);
    auto dec(RM)(RM dst) if (valid!(RM, 64)) => emit!1(0xff, dst);

    static if (!X64)
    auto dec(R16 dst) => emit!(0, NRM)(0x48, dst);
    static if (!X64)
    auto dec(R32 dst) => emit!(0, NRM)(0x48, dst);

    auto int3() => emit!0(0xcc);
    auto _int(ubyte imm8) => emit!0(0xcd, imm8);
    auto into() => emit!0(0xce);
    auto int1() => emit!0(0xf1);
    auto ud0(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xff, dst, src);
    auto ud1(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xb9, dst, src);
    auto ud2() => emit!0(0x0f, 0x0b);
    
    auto iret() => emit!0(0xcf);
    auto iretd() => emit!0(0xcf);
    auto iretq() => emit!0(0xcf);

    auto inc(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0xfe, dst);
    static if (X64)
    auto inc(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0xff, dst);
    static if (!X64)
    auto inc(Address!16 dst) => emit!0(0xff, dst);
    static if (X64)
    auto inc(RM)(RM dst) if (valid!(RM, 32)) => emit!0(0xff, dst);
    static if (!X64)
    auto inc(Address!32 dst) => emit!0(0xff, dst);
    auto inc(RM)(RM dst) if (valid!(RM, 64)) => emit!0(0xff, dst);

    static if (!X64)
    auto inc(R16 dst) => emit!(0, NRM)(0x40, dst);
    static if (!X64)
    auto inc(R32 dst) => emit!(0, NRM)(0x40, dst);

    auto hlt() => emit!0(0xf4);
    auto pause() => emit!0(0xf3, 0x90);
    auto swapgs() => emit!0(0x0f, 0x01, 0xf8);
    
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
    auto enter(ushort imm16) => emit!0(0xc8, imm16, 0x00);
    auto enter(ushort imm16, ubyte imm8) => emit!0(0xc8, imm16, imm8);
    
    auto lea(RM)(R16 dst, Address!16) => emit!0(0x8d, dst, src);
    auto lea(RM)(R32 dst, Address!32) => emit!0(0x8d, dst, src);
    auto lea(RM)(R64 dst, Address!64) => emit!0(0x8d, dst, src);

    auto lds(RM)(R16 dst, Address!16) => emit!0(0xc5, dst, src);
    auto lds(RM)(R32 dst, Address!32) => emit!0(0xc5, dst, src);

    auto lss(RM)(R16 dst, Address!16) => emit!0(0x0f, 0xb2, dst, src);
    auto lss(RM)(R32 dst, Address!32) => emit!0(0x0f, 0xb2, dst, src);
    auto lss(RM)(R64 dst, Address!64) => emit!0(0x0f, 0xb2, dst, src);

    auto les(RM)(R16 dst, Address!16) => emit!0(0xc4, dst, src);
    auto les(RM)(R32 dst, Address!32) => emit!0(0xc4, dst, src);

    auto lfs(RM)(R16 dst, Address!16) => emit!0(0x0f, 0xb4, dst, src);
    auto lfs(RM)(R32 dst, Address!32) => emit!0(0x0f, 0xb4, dst, src);
    auto lfs(RM)(R64 dst, Address!64) => emit!0(0x0f, 0xb4, dst, src);

    auto lgs(RM)(R16 dst, Address!16) => emit!0(0x0f, 0xb5, dst, src);
    auto lgs(RM)(R32 dst, Address!32) => emit!0(0x0f, 0xb5, dst, src);
    auto lgs(RM)(R64 dst, Address!64) => emit!0(0x0f, 0xb5, dst, src);

    auto lsl(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0x03, dst, src);
    auto lsl(R32 dst, R32 src) => emit!0(0x0f, 0x03, dst, src);
    auto lsl(R64 dst, R32 src) => emit!0(0x0f, 0x03, dst, src);
    auto lsl(R32 dst, Address!16 src) => emit!0(0x0f, 0x03, dst, src);
    auto lsl(R64 dst, Address!16 src) => emit!0(0x0f, 0x03, dst, src);

    auto ltr(RM)(RM dst) if (valid!(RM, 16)) => emit!3(0x0f, 0x00, dst);
    auto str(RM)(RM dst) if (valid!(RM, 16)) => emit!1(0x0f, 0x00, dst);

    auto neg(RM)(RM dst) if (valid!(RM, 8)) => emit!3(0xf6, dst);
    auto neg(RM)(RM dst) if (valid!(RM, 16)) => emit!3(0xf7, dst);
    auto neg(RM)(RM dst) if (valid!(RM, 32)) => emit!3(0xf7, dst);
    auto neg(RM)(RM dst) if (valid!(RM, 64)) => emit!3(0xf7, dst);

    auto nop() => emit!0(0x90);
    auto nop(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0x0f, 0x1f, dst);
    auto nop(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0x0f, 0x1f, dst);

    auto not(RM)(RM dst) if (valid!(RM, 8)) => emit!2(0xf6, dst);
    auto not(RM)(RM dst) if (valid!(RM, 16)) => emit!2(0xf7, dst);
    auto not(RM)(RM dst) if (valid!(RM, 32)) => emit!2(0xf7, dst);
    auto not(RM)(RM dst) if (valid!(RM, 64)) => emit!2(0xf7, dst);

    auto ret() => emit!0(0xc3);
    auto ret(ushort imm16) => emit!0(0xc2, imm16);
    auto retf() => emit!0(0xcb);
    auto retf(ushort imm16) => emit!0(0xca, imm16);

    auto stc() => emit!0(0xf9);
    auto std() => emit!0(0xfd);
    auto sti() => emit!0(0xfb);

    auto sub(ubyte imm8) => emit!0(0x2c, imm8);
    auto sub(ushort imm16) => emit!0(0x2d, imm16);
    auto sub(uint imm32) => emit!0(0x2d, imm32);
    auto sub(ulong imm32) => emit!0(0x2d, cast(long)imm32);

    auto sub(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!5(0x80, dst, imm8);
    auto sub(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!5(0x81, dst, imm16);
    auto sub(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!5(0x81, dst, imm32);
    auto sub(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!5(0x81, dst, imm32);
    auto sub(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!5(0x83, dst, imm8);
    auto sub(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!5(0x83, dst, imm8);
    auto sub(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!5(0x83, dst, imm8);

    auto sub(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x28, dst, src);
    auto sub(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x29, dst, src);
    auto sub(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x29, dst, src);
    auto sub(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x29, dst, src);

    auto sub(R8 dst, Address!8 src) => emit!0(0x2a, dst, src);
    auto sub(R16 dst, Address!16 src) => emit!0(0x2b, dst, src);
    auto sub(R32 dst, Address!32 src) => emit!0(0x2b, dst, src);
    auto sub(R64 dst, Address!64 src) => emit!0(0x2b, dst, src);

    auto sbb(ubyte imm8) => emit!0(0x1c, imm8);
    auto sbb(ushort imm16) => emit!0(0x1d, imm16);
    auto sbb(uint imm32) => emit!0(0x1d, imm32);
    auto sbb(ulong imm32) => emit!0(0x1d, cast(long)imm32);

    auto sbb(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!3(0x80, dst, imm8);
    auto sbb(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!3(0x81, dst, imm16);
    auto sbb(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!3(0x81, dst, imm32);
    auto sbb(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!3(0x81, dst, imm32);
    auto sbb(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!3(0x83, dst, imm8);
    auto sbb(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!3(0x83, dst, imm8);
    auto sbb(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!3(0x83, dst, imm8);

    auto sbb(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x18, dst, src);
    auto sbb(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x19, dst, src);
    auto sbb(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x19, dst, src);
    auto sbb(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x19, dst, src);

    auto sbb(R8 dst, Address!8 src) => emit!0(0x1a, dst, src);
    auto sbb(R16 dst, Address!16 src) => emit!0(0x1b, dst, src);
    auto sbb(R32 dst, Address!32 src) => emit!0(0x1b, dst, src);
    auto sbb(R64 dst, Address!64 src) => emit!0(0x1b, dst, src);

    auto xor(ubyte imm8) => emit!0(0x34, imm8);
    auto xor(ushort imm16) => emit!0(0x35, imm16);
    auto xor(uint imm32) => emit!0(0x35, imm32);
    auto xor(ulong imm32) => emit!0(0x35, cast(long)imm32);

    auto xor(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!6(0x80, dst, imm8);
    auto xor(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!6(0x81, dst, imm16);
    auto xor(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!6(0x81, dst, imm32);
    auto xor(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!6(0x81, dst, imm32);
    auto xor(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!6(0x83, dst, imm8);
    auto xor(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!6(0x83, dst, imm8);
    auto xor(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!6(0x83, dst, imm8);

    auto xor(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x30, dst, src);
    auto xor(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x31, dst, src);
    auto xor(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x31, dst, src);
    auto xor(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x31, dst, src);

    auto xor(R8 dst, Address!8 src) => emit!0(0x32, dst, src);
    auto xor(R16 dst, Address!16 src) => emit!0(0x33, dst, src);
    auto xor(R32 dst, Address!32 src) => emit!0(0x33, dst, src);
    auto xor(R64 dst, Address!64 src) => emit!0(0x33, dst, src);

    auto or(ubyte imm8) => emit!0(0x0c, imm8);
    auto or(ushort imm16) => emit!0(0x0d, imm16);
    auto or(uint imm32) => emit!0(0x0d, imm32);
    auto or(ulong imm32) => emit!0(0x0d, cast(long)imm32);

    auto or(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!1(0x80, dst, imm8);
    auto or(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!1(0x81, dst, imm16);
    auto or(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!1(0x81, dst, imm32);
    auto or(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!1(0x81, dst, imm32);
    auto or(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => emit!1(0x83, dst, imm8);
    auto or(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => emit!1(0x83, dst, imm8);
    auto or(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => emit!1(0x83, dst, imm8);

    auto or(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x8, dst, src);
    auto or(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x9, dst, src);
    auto or(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x9, dst, src);
    auto or(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x9, dst, src);

    auto or(R8 dst, Address!8 src) => emit!0(0xa, dst, src);
    auto or(R16 dst, Address!16 src) => emit!0(0xb, dst, src);
    auto or(R32 dst, Address!32 src) => emit!0(0xb, dst, src);
    auto or(R64 dst, Address!64 src) => emit!0(0xb, dst, src);

    auto sal(RM)(RM dst) if (valid!(RM, 8)) => emit!4(0xd2, dst, cl);
    auto sal(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!4(0xd0, dst);
        else
            return emit!4(0xc0, dst, imm8);
    }
    auto sal(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0xd3, dst, cl);
    auto sal(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!4(0xd1, dst);
        else
            return emit!4(0xc1, dst, imm8);
    }
    auto sal(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0xd3, dst, cl);
    auto sal(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!4(0xd1, dst);
        else
            return emit!4(0xc1, dst, imm8);
    }
    auto sal(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0xd3, dst, cl);
    auto sal(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!4(0xd1, dst);
        else
            return emit!4(0xc1, dst, imm8);
    }

    auto sar(RM)(RM dst) if (valid!(RM, 8)) => emit!7(0xd2, dst, cl);
    auto sar(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!7(0xd0, dst);
        else
            return emit!7(0xc0, dst, imm8);
    }
    auto sar(RM)(RM dst) if (valid!(RM, 16)) => emit!7(0xd3, dst, cl);
    auto sar(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!7(0xd1, dst);
        else
            return emit!7(0xc1, dst, imm8);
    }
    auto sar(RM)(RM dst) if (valid!(RM, 32)) => emit!7(0xd3, dst, cl);
    auto sar(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!7(0xd1, dst);
        else
            return emit!7(0xc1, dst, imm8);
    }
    auto sar(RM)(RM dst) if (valid!(RM, 64)) => emit!7(0xd3, dst, cl);
    auto sar(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!7(0xd1, dst);
        else
            return emit!7(0xc1, dst, imm8);
    }

    auto shl(RM)(RM dst) if (valid!(RM, 8)) => sal(dst);
    auto shl(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => sal(dst, imm8);
    auto shl(RM)(RM dst) if (valid!(RM, 16)) => sal(dst);
    auto shl(RM)(RM dst, ubyte imm8) if (valid!(RM, 16)) => sal(dst, imm8);
    auto shl(RM)(RM dst) if (valid!(RM, 32)) => sal(dst);
    auto shl(RM)(RM dst, ubyte imm8) if (valid!(RM, 32)) => sal(dst, imm8);
    auto shl(RM)(RM dst) if (valid!(RM, 64)) => sal(dst);
    auto shl(RM)(RM dst, ubyte imm8) if (valid!(RM, 64)) => sal(dst, imm8);

    auto shr(RM)(RM dst) if (valid!(RM, 8)) => emit!5(0xd2, dst, cl);
    auto shr(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!5(0xd0, dst);
        else
            return emit!5(0xc0, dst, imm8);
    }
    auto shr(RM)(RM dst) if (valid!(RM, 16)) => emit!5(0xd3, dst, cl);
    auto shr(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!5(0xd1, dst);
        else
            return emit!5(0xc1, dst, imm8);
    }
    auto shr(RM)(RM dst) if (valid!(RM, 32)) => emit!5(0xd3, dst, cl);
    auto shr(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!5(0xd1, dst);
        else
            return emit!5(0xc1, dst, imm8);
    }
    auto shr(RM)(RM dst) if (valid!(RM, 64)) => emit!5(0xd3, dst, cl);
    auto shr(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!5(0xd1, dst);
        else
            return emit!5(0xc1, dst, imm8);
    }

    auto rcl(RM)(RM dst) if (valid!(RM, 8)) => emit!2(0xd2, dst, cl);
    auto rcl(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!2(0xd0, dst);
        else
            return emit!2(0xc0, dst, imm8);
    }
    auto rcl(RM)(RM dst) if (valid!(RM, 16)) => emit!2(0xd3, dst, cl);
    auto rcl(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!2(0xd1, dst);
        else
            return emit!2(0xc1, dst, imm8);
    }
    auto rcl(RM)(RM dst) if (valid!(RM, 32)) => emit!2(0xd3, dst, cl);
    auto rcl(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!2(0xd1, dst);
        else
            return emit!2(0xc1, dst, imm8);
    }
    auto rcl(RM)(RM dst) if (valid!(RM, 64)) => emit!2(0xd3, dst, cl);
    auto rcl(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!2(0xd1, dst);
        else
            return emit!2(0xc1, dst, imm8);
    }

    auto rcr(RM)(RM dst) if (valid!(RM, 8)) => emit!3(0xd2, dst, cl);
    auto rcr(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!3(0xd0, dst);
        else
            return emit!3(0xc0, dst, imm8);
    }
    auto rcr(RM)(RM dst) if (valid!(RM, 16)) => emit!3(0xd3, dst, cl);
    auto rcr(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!3(0xd1, dst);
        else
            return emit!3(0xc1, dst, imm8);
    }
    auto rcr(RM)(RM dst) if (valid!(RM, 32)) => emit!3(0xd3, dst, cl);
    auto rcr(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!3(0xd1, dst);
        else
            return emit!3(0xc1, dst, imm8);
    }
    auto rcr(RM)(RM dst) if (valid!(RM, 64)) => emit!3(0xd3, dst, cl);
    auto rcr(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!3(0xd1, dst);
        else
            return emit!3(0xc1, dst, imm8);
    }

    auto rol(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0xd2, dst, cl);
    auto rol(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!0(0xd0, dst);
        else
            return emit!0(0xc0, dst, imm8);
    }
    auto rol(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0xd3, dst, cl);
    auto rol(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!0(0xd1, dst);
        else
            return emit!0(0xc1, dst, imm8);
    }
    auto rol(RM)(RM dst) if (valid!(RM, 32)) => emit!0(0xd3, dst, cl);
    auto rol(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!0(0xd1, dst);
        else
            return emit!0(0xc1, dst, imm8);
    }
    auto rol(RM)(RM dst) if (valid!(RM, 64)) => emit!0(0xd3, dst, cl);
    auto rol(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!0(0xd1, dst);
        else
            return emit!0(0xc1, dst, imm8);
    }

    auto ror(RM)(RM dst) if (valid!(RM, 8)) => emit!1(0xd2, dst, cl);
    auto ror(RM)(RM dst, ubyte imm8) if (valid!(RM, 8))
    {
        if (imm8 == 1)
            return emit!1(0xd0, dst);
        else
            return emit!1(0xc0, dst, imm8);
    }
    auto ror(RM)(RM dst) if (valid!(RM, 16)) => emit!1(0xd3, dst, cl);
    auto ror(RM)(RM dst, ubyte imm8) if (valid!(RM, 16))
    {
        if (imm8 == 1)
            return emit!1(0xd1, dst);
        else
            return emit!1(0xc1, dst, imm8);
    }
    auto ror(RM)(RM dst) if (valid!(RM, 32)) => emit!1(0xd3, dst, cl);
    auto ror(RM)(RM dst, ubyte imm8) if (valid!(RM, 32))
    {
        if (imm8 == 1)
            return emit!1(0xd1, dst);
        else
            return emit!1(0xc1, dst, imm8);
    }
    auto ror(RM)(RM dst) if (valid!(RM, 64)) => emit!1(0xd3, dst, cl);
    auto ror(RM)(RM dst, ubyte imm8) if (valid!(RM, 64))
    {
        if (imm8 == 1)
            return emit!1(0xd1, dst);
        else
            return emit!1(0xc1, dst, imm8);
    }

    auto verr(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0xf0, 0x00, dst);
    auto verw(RM)(RM dst) if (valid!(RM, 16)) => emit!5(0xf0, 0x00, dst);

    auto test(ubyte imm8) => emit!0(0xa8, imm8);
    auto test(ushort imm16) => emit!0(0xa9, imm16);
    auto test(uint imm32) => emit!0(0xa9, imm32);
    auto test(ulong imm32) => emit!0(0xa9, cast(long)imm32);

    auto test(RM)(RM dst, ubyte imm8) if (valid!(RM, 8)) => emit!0(0xf6, dst, imm8);
    auto test(RM)(RM dst, ushort imm16) if (valid!(RM, 16)) => emit!0(0xf7, dst, imm16);
    auto test(RM)(RM dst, uint imm32) if (valid!(RM, 32)) => emit!0(0xf7, dst, imm32);
    auto test(RM)(RM dst, uint imm32) if (valid!(RM, 64)) => emit!0(0xf7, dst, cast(long)imm32);

    auto test(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x84, dst, src);
    auto test(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x85, dst, src);
    auto test(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x85, dst, src);
    auto test(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x85, dst, src);

    auto pop(RM)(RM dst) if (valid!(RM, 16)) => emit!0(0x8f, dst);
    static if (!X64)
    auto pop(RM)(RM dst) if (valid!(RM, 32)) => emit!0(0x8f, dst);
    static if (X64)
    auto pop(RM)(RM dst) if (valid!(RM, 64)) => emit!0(0x8f, dst);

    auto pop(R16 dst) => emit!(0, NRM)(0x58, dst);
    static if (!X64)
    auto pop(R32 dst) => emit!(0, NRM)(0x58, dst);
    static if (X64)
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

    auto push(RM)(RM dst) if (valid!(RM, 16)) => emit!6(0xff, dst);
    static if (!X64)
    auto push(RM)(RM dst) if (valid!(RM, 32)) => emit!6(0xff, dst);
    static if (X64)
    auto push(RM)(RM dst) if (valid!(RM, 64)) => emit!6(0xff, dst);

    auto push(R16 dst) => emit!(0, NRM)(0x50, dst);
    static if (!X64)
    auto push(R32 dst) => emit!(0, NRM)(0x50, dst);
    static if (X64)
    auto push(R64 dst) => emit!(0, NRM)(0x50, dst);

    auto push(ubyte imm8) => emit!0(0x6a, imm8);
    auto push(ushort imm16) => emit!0(0x68, imm16);
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

    auto xadd(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x0f, 0xc0, dst, src);
    auto xadd(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x0f, 0xc1, dst, src);
    auto xadd(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x0f, 0xc1, dst, src);
    auto xadd(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x0f, 0xc1, dst, src);

    auto xchg(R16 dst) => emit!(0, NRM)(90, dst);
    auto xchg(R32 dst) => emit!(0, NRM)(90, dst);
    auto xchg(R64 dst) => emit!(0, NRM)(90, dst);

    auto xchg(A, B)(A dst, B src) if (valid!(A, 8) && valid!(B, 8)) => emit!0(0x86, dst, src);
    auto xchg(A, B)(A dst, B src) if (valid!(A, 16) && valid!(B, 16)) => emit!0(0x87, dst, src);
    auto xchg(A, B)(A dst, B src) if (valid!(A, 32) && valid!(B, 32)) => emit!0(0x87, dst, src);
    auto xchg(A, B)(A dst, B src) if (valid!(A, 64) && valid!(B, 64)) => emit!0(0x87, dst, src);

    auto xlat() => emit!0(0xd7);
    static if (!X64)
    auto xlatb() => emit!0(0xd7);
    static if (X64)
    auto xlatb() => emit!0(0x48, 0xd7);

    auto lar(R16 dst, Address!16 src) => emit!0(0x0f, 0x02, dst, src);
    auto lar(R16 dst, R16 src) => emit!0(0x0f, 0x02, dst, src);
    auto lar(R32 dst, Address!16 src) => emit!0(0x0f, 0x02, dst, src);
    auto lar(R32 dst, R32 src) => emit!0(0x0f, 0x02, dst, src);

    auto daa() => emit!0(0x27);
    auto das() => emit!0(0x2f);

    auto mul(RM)(RM dst) if (valid!(RM, 8)) => emit!4(0xf6, dst);
    auto mul(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0xf7, dst);
    auto mul(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0xf7, dst);
    auto mul(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0xf7, dst);

    auto imul(RM)(RM dst) if (valid!(RM, 8)) => emit!5(0xf6, dst);
    auto imul(RM)(RM dst) if (valid!(RM, 16)) => emit!5(0xf7, dst);
    auto imul(RM)(RM dst) if (valid!(RM, 32)) => emit!5(0xf7, dst);
    auto imul(RM)(RM dst) if (valid!(RM, 64)) => emit!5(0xf7, dst);

    auto imul(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xaf, dst, src);
    auto imul(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x0f, 0xaf, dst, src);
    auto imul(RM)(R64 dst, RM src) if (valid!(RM, 64)) => emit!0(0x0f, 0xaf, dst, src);

    auto imul(RM)(R16 dst, RM src, ubyte imm8) if (valid!(RM, 16)) => emit!0(0x6b, dst, src, imm8);
    auto imul(RM)(R32 dst, RM src, ubyte imm8) if (valid!(RM, 32)) => emit!0(0x6b, dst, src, imm8);
    auto imul(RM)(R64 dst, RM src, ubyte imm8) if (valid!(RM, 64)) => emit!0(0x6b, dst, src, imm8);
    auto imul(RM)(R16 dst, RM src, ushort imm16) if (valid!(RM, 16)) => emit!0(0x69, dst, src, imm16);
    auto imul(RM)(R32 dst, RM src, uint imm32) if (valid!(RM, 32)) => emit!0(0x69, dst, src, imm32);
    auto imul(RM)(R64 dst, RM src, uint imm32) if (valid!(RM, 64)) => emit!0(0x69, dst, src, imm32);

    auto div(RM)(RM dst) if (valid!(RM, 8)) => emit!6(0xf6, dst);
    auto div(RM)(RM dst) if (valid!(RM, 16)) => emit!6(0xf7, dst);
    auto div(RM)(RM dst) if (valid!(RM, 32)) => emit!6(0xf7, dst);
    auto div(RM)(RM dst) if (valid!(RM, 64)) => emit!6(0xf7, dst);

    auto idiv(RM)(RM dst) if (valid!(RM, 8)) => emit!7(0xf6, dst);
    auto idiv(RM)(RM dst) if (valid!(RM, 16)) => emit!7(0xf7, dst);
    auto idiv(RM)(RM dst) if (valid!(RM, 32)) => emit!7(0xf7, dst);
    auto idiv(RM)(RM dst) if (valid!(RM, 64)) => emit!7(0xf7, dst);

    auto mov(RM)(RM dst, R8 src) if (valid!(RM, 8)) => emit!0(0x88, dst, src);
    auto mov(RM)(RM dst, R16 src) if (valid!(RM, 16)) => emit!0(0x89, dst, src);
    auto mov(RM)(RM dst, R32 src) if (valid!(RM, 32)) => emit!0(0x89, dst, src);
    auto mov(RM)(RM dst, R64 src) if (valid!(RM, 64)) => emit!0(0x89, dst, src);

    auto mov(R8 dst, Address!8 src) => emit!0(0x8a, dst, src);
    auto mov(R16 dst, Address!16 src) => emit!0(0x8b, dst, src);
    auto mov(R32 dst, Address!32 src) => emit!0(0x8b, dst, src);
    auto mov(R64 dst, Address!64 src) => emit!0(0x8b, dst, src);
    
    auto mov(R8 dst, ubyte imm8) => emit!(0, NRM)(0xb0, dst, imm8);
    auto mov(R16 dst, ushort imm16) => emit!(0, NRM)(0xb8, dst, imm16);
    auto mov(R32 dst, uint imm32) => emit!(0, NRM)(0xb8, dst, imm32);
    auto mov(R64 dst, ulong imm64) => emit!(0, NRM)(0xb8, dst, imm64);

    auto mov(Address!8 dst, ubyte imm8) => emit!0(0xc6, dst, imm8);
    auto mov(Address!16 dst, ushort imm16) => emit!0(0xc7, dst, imm16);
    auto mov(Address!32 dst, uint imm32) => emit!0(0xc7, dst, imm32);
    auto mov(Address!64 dst, uint imm32) => emit!0(0xc7, dst, imm32);

    auto mov(R32 dst, Reg!(-1) src) => emit!0(0x0f, 0x20, dst, src);
    auto mov(R64 dst, Reg!(-1) src) => emit!0(0x0f, 0x20, dst, src);
    auto mov(Reg!(-1) dst, R32 src) => emit!0(0x0f, 0x22, dst, src);
    auto mov(Reg!(-1) dst, R64 src) => emit!0(0x0f, 0x22, dst, src);

    auto mov(R32 dst, Reg!(-2) src) => emit!0(0x0f, 0x21, dst, src);
    auto mov(R64 dst, Reg!(-2) src) => emit!0(0x0f, 0x21, dst, src);
    auto mov(Reg!(-2) dst, R32 src) => emit!0(0x0f, 0x23, dst, src);
    auto mov(Reg!(-2) dst, R64 src) => emit!0(0x0f, 0x23, dst, src);

    auto movsx(RM)(R16 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xbe, dst, src);
    auto movsx(RM)(R32 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xbe, dst, src);
    auto movsx(RM)(R64 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xbe, dst, src);

    auto movsx(RM)(R32 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbf, dst, src);
    auto movsx(RM)(R64 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xbf, dst, src);

    auto movsxd(RM)(R16 dst, RM src) if (valid!(RM, 16)) => emit!0(0x63, dst, src);
    auto movsxd(RM)(R32 dst, RM src) if (valid!(RM, 32)) => emit!0(0x63, dst, src);
    auto movsxd(RM)(R64 dst, RM src) if (valid!(RM, 32)) => emit!0(0x63, dst, src);

    auto movzx(RM)(R16 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xb6, dst, src);
    auto movzx(RM)(R32 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xb6, dst, src);
    auto movzx(RM)(R64 dst, RM src) if (valid!(RM, 8)) => emit!0(0x0f, 0xb6, dst, src);

    auto movzx(RM)(R32 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xb7, dst, src);
    auto movzx(RM)(R64 dst, RM src) if (valid!(RM, 16)) => emit!0(0x0f, 0xb7, dst, src);

    auto call(ushort rel16) => emit!0(0xe8, rel16);
    auto call(uint rel32) => emit!0(0xe8, rel32);

    auto call(R16 dst) => emit!2(0xff, dst);
    auto call(R32 dst) => emit!2(0xff, dst);
    auto call(R64 dst) => emit!2(0xff, dst);

    auto call(Address!16 dst) => emit!3(0xff, dst);
    auto call(Address!32 dst) => emit!3(0xff, dst);
    auto call(Address!64 dst) => emit!3(0xff, dst);

    auto loop(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loop", name !in labels);
    auto loope(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loope", name !in labels);
    auto loopne(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "loopne", name !in labels);

    auto jmp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jmp", name !in labels);
    auto jmp(RM)(RM dst) if (valid!(RM, 16)) => emit!4(0xff, dst);
    auto jmp(RM)(RM dst) if (valid!(RM, 32)) => emit!4(0xff, dst);
    auto jmp(RM)(RM dst) if (valid!(RM, 64)) => emit!4(0xff, dst);

    auto jmp(Address!16 dst) => emit!5(0xff, dst);
    auto jmp(Address!32 dst) => emit!5(0xff, dst);
    auto jmp(Address!64 dst) => emit!5(0xff, dst);

    auto jmp(ushort imm16) => emit!0(0xea, imm16);
    auto jmp(uint imm32) => emit!0(0xea, imm32);

    auto ja(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "ja", name !in labels);
    auto jae(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jae", name !in labels);
    auto jb(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jb", name !in labels);
    auto jbe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jbe", name !in labels);
    auto jc(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jc", name !in labels);
    auto jcxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jcxz", name !in labels);
    auto jecxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jecxz", name !in labels);
    auto jrcxz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jrcxz", name !in labels);
    auto je(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "je", name !in labels);
    auto jg(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jg", name !in labels);
    auto jge(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jge", name !in labels);
    auto jl(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jl", name !in labels);
    auto jle(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jle", name !in labels);
    auto jna(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jna", name !in labels);
    auto jnae(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnae", name !in labels);
    auto jnb(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnb", name !in labels);
    auto jnbe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnbe", name !in labels);
    auto jnc(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnc", name !in labels);
    auto jne(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jne", name !in labels);
    auto jng(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jng", name !in labels);
    auto jnge(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnge", name !in labels);
    auto jnl(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnl", name !in labels);
    auto jnle(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnle", name !in labels);
    auto jno(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jno", name !in labels);
    auto jnp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnp", name !in labels);
    auto jns(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jns", name !in labels);
    auto jnz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jnz", name !in labels);
    auto jo(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jo", name !in labels);
    auto jp(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jp", name !in labels);
    auto jpe(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jpe", name !in labels);
    auto jpo(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jpo", name !in labels);
    auto js(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "js", name !in labels);
    auto jz(string name) => branches ~= tuple(cast(ptrdiff_t)buffer.length, name, "jz", name !in labels);
        
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

    auto movs(Address!8 dst, Address!8 src) => emit!0(0xa4, dst, src);
    auto movs(Address!16 dst, Address!16 src) => emit!0(0xa5, dst, src);
    auto movs(Address!32 dst, Address!32 src) => emit!0(0xa5, dst, src);
    auto movs(Address!64 dst, Address!64 src) => emit!0(0xa5, dst, src);

    auto movsb() => emit!0(0xa4);
    auto movsw() => emit!0(0x66, 0xa5);
    auto movsd() => emit!0(0xa5);
    auto movsq() => emit!0(0x48, 0xa5);

    auto cmps(Address!8 dst, Address!8 src) => emit!0(0xa6, dst, src);
    auto cmps(Address!16 dst, Address!16 src) => emit!0(0xa7, dst, src);
    auto cmps(Address!32 dst, Address!32 src) => emit!0(0xa7, dst, src);
    auto cmps(Address!64 dst, Address!64 src) => emit!0(0xa7, dst, src);

    auto cmpsb() => emit!0(0xa6);
    auto cmpsw() => emit!0(0x66, 0xa7);
    auto cmpsd() => emit!0(0xa7);
    auto cmpsq() => emit!0(0x48, 0xa7);

    auto scas(Address!8 dst) => emit!0(0xae, dst);
    auto scas(Address!16 dst) => emit!0(0xaf, dst);
    auto scas(Address!32 dst) => emit!0(0xaf, dst);
    auto scas(Address!64 dst) => emit!0(0xaf, dst);

    auto scasb() => emit!0(0xae);
    auto scasw() => emit!0(0x66, 0xaf);
    auto scasd() => emit!0(0xaf);
    auto scasq() => emit!0(0x48, 0xaf);

    auto lods(Address!8 dst) => emit!0(0xac, dst);
    auto lods(Address!16 dst) => emit!0(0xad, dst);
    auto lods(Address!32 dst) => emit!0(0xad, dst);
    auto lods(Address!64 dst) => emit!0(0xad, dst);

    auto lodsb() => emit!0(0xac);
    auto lodsw() => emit!0(0x66, 0xad);
    auto lodsd() => emit!0(0xad);
    auto lodsq() => emit!0(0x48, 0xad);

    auto stos(Address!8 dst) => emit!0(0xaa, dst);
    auto stos(Address!16 dst) => emit!0(0xab, dst);
    auto stos(Address!32 dst) => emit!0(0xab, dst);
    auto stos(Address!64 dst) => emit!0(0xab, dst);

    auto stosb() => emit!0(0xaa);
    auto stosw() => emit!0(0x66, 0xab);
    auto stosd() => emit!0(0xab);
    auto stosq() => emit!0(0x48, 0xab);

    auto inal(ubyte imm8) => emit!0(0xe4, imm8);
    auto _in(ubyte imm8) => emit!0(0xe5, imm8);
    auto inal() => emit!0(0xec);
    auto _in() => emit!0(0xed);

    auto ins(Address!8 dst) => emit!0(0x6c, dst);
    auto ins(Address!16 dst) => emit!0(0x6d, dst);
    auto ins(Address!32 dst) => emit!0(0x6d, dst);

    auto insb() => emit!0(0x6c);
    auto insw() => emit!0(0x66, 0x6d);
    auto insd() => emit!0(0x6d);
    
    auto outal(ubyte imm8) => emit!0(0xe6, imm8);
    auto _out(ubyte imm8) => emit!0(0xe7, imm8);
    auto outal() => emit!0(0xee);
    auto _out() => emit!0(0xef);

    auto outs(Address!8 dst) => emit!0(0x6e, dst);
    auto outs(Address!16 dst) => emit!0(0x6f, dst);
    auto outs(Address!32 dst) => emit!0(0x6f, dst);

    auto outsb() => emit!0(0x6e);
    auto outsw() => emit!0(0x66, 0x6f);
    auto outsd() => emit!0(0x6f);

    auto seta(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x97, dst);
    auto setae(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x93, dst);
    auto setb(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x92, dst);
    auto setbe(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x96, dst);
    auto setc(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x92, dst);
    auto sete(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x94, dst);
    auto setg(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9f, dst);
    auto setge(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9d, dst);
    auto setl(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9c, dst);
    auto setle(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9e, dst);
    auto setna(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x96, dst);
    auto setnae(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x92, dst);
    auto setnb(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x93, dst);
    auto setnbe(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x97, dst);
    auto setnc(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x93, dst);
    auto setne(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x95, dst);
    auto setng(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9e, dst);
    auto setnge(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9c, dst);
    auto setnl(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9d, dst);
    auto setnle(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9f, dst);
    auto setno(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x91, dst);
    auto setnp(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9b, dst);
    auto setns(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x99, dst);
    auto setnz(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x95, dst);
    auto seto(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x90, dst);
    auto setp(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9a, dst);
    auto setpe(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9a, dst);
    auto setpo(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x9b, dst);
    auto sets(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x98, dst);
    auto setz(RM)(RM dst) if (valid!(RM, 8)) => emit!0(0x0f, 0x94, dst);

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