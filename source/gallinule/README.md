ASMI is the assembly representation standardized for Gallinule.

- [x] x86 - x86_64
- [ ] ARM64 - ARM
- [ ] CIL

```
    0x4a 0x00 $0 0xf3 $1 mnemonic r32 imm8

    vex128.i [xop] 0xf2 0x88 $0 $1 $2 mnemonic r128 imm rm128

    mvex256 0xf3p 0x00 $0 $1 mnemonic m64 rm64
```

Representation of an instruction should be done on a single line, with the number of references equal to or greater than the number of parameters. Pseudo-instructions may be added by simply writing D code instead of emission data, this is limited to calling functions, like `cpuid(1) shr(edx, 3) mnemonic` to create a pseudo-instruction.

Mnemonic must come before parameters and pseudo-instuction combos, all other representation items may appear at any point in a line, so long as they conform to the following table.

| Item | Representation |
|--------|----------------|
| Mnemonic | No special representation. |
| Combo | No special representation. |
| Reference | `$n` |
| Register | `r*`  **appended by decimal bit size or `xx`.* |
| Memory | `m*`  **appended by decimal bit size or `xx`.* |
| Register (or) Memory | `rm*` **appended by decimal bit size or `xx`.* |
| Immediate | `i*`  **appended by decimal bit size or `xx`.* |
| ModRM OR | `/n*` **where `n` is decimal* |
| Hexadecimal | `0xnn` |
| MVEX | `mvex{size}[.i]*` **will make the prefix integral-kind.* |
| EVEX | `evex{size}[.i]*` **will make the prefix integral-kind.* |
| VEX | `vex{size}[.i]*` **will make the prefix integral-kind.* |
| AVX | `avx{size}[.i]*` **will make the prefix integral-kind.* |
| VEX Prefix Field | `0xnnp` |
| VEX Map Field | `[m*]` **where `m` is any map* |
| Toggle REX Off | `/r` |
| Toggle Register Flipping On | `/f` |
| Toggle Direct Encoding On | `/e` |

| VEX Maps |
|----------|
| XOP |
| 38 |
| 3A |
| MSR |