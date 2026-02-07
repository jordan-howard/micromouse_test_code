.syntax unified
.thumb
.cpu cortex-m3

/* ------------------------------------------------------------
 * Branch-heavy benchmark
 * R0 = iterations
 * Returns R0 = final LFSR state (prevents optimizing away)
 * ------------------------------------------------------------ */

.global branch_bench_flash
.type   branch_bench_flash, %function
branch_bench_flash:
    push {r4, lr}
    ldr  r4, =0xA5A5A5A5          /* LFSR seed in r4 */
1:
    /* --- LFSR update (xorshift-ish) --- */
    eor  r4, r4, r4, lsl #13
    eor  r4, r4, r4, lsr #17
    eor  r4, r4, r4, lsl #5

    /* --- Branch storm: lots of unpredictable conditionals --- */
    tst  r4, #0x00000001
    bne  2f
    tst  r4, #0x00000002
    bne  3f
    tst  r4, #0x00000004
    bne  4f
    tst  r4, #0x00000008
    bne  5f
    tst  r4, #0x00000010
    bne  6f
    tst  r4, #0x00000020
    bne  7f
    tst  r4, #0x00000040
    bne  8f
    tst  r4, #0x00000080
    bne  9f

    /* fallthrough path (also does work) */
    add  r4, r4, #1
    b    10f

2:  sub  r4, r4, #3
    b    10f
3:  add  r4, r4, #7
    b    10f
4:  eor  r4, r4, #0x11
    b    10f
5:  add  r4, r4, r4, ror #3
    b    10f
6:  sub  r4, r4, r4, lsr #2
    b    10f
7:  eor  r4, r4, r4, lsr #1
    b    10f
8:  add  r4, r4, #0x123
    b    10f
9:  sub  r4, r4, #0x55

10:
    subs r0, r0, #1
    bne  1b

    mov  r0, r4
    pop  {r4, pc}

.size branch_bench_flash, .-branch_bench_flash


/* ------------------------------------------------------------
 * Same exact function, but placed in SRAM section (.RamFunc)
 * Your linker/startup must copy .RamFunc to RAM at boot.
 * ------------------------------------------------------------ */
.section .RamFunc, "ax", %progbits
.global branch_bench_ram
.type   branch_bench_ram, %function
branch_bench_ram:
    push {r4, lr}
    ldr  r4, =0xA5A5A5A5
1:
    eor  r4, r4, r4, lsl #13
    eor  r4, r4, r4, lsr #17
    eor  r4, r4, r4, lsl #5

    tst  r4, #0x00000001
    bne  2f
    tst  r4, #0x00000002
    bne  3f
    tst  r4, #0x00000004
    bne  4f
    tst  r4, #0x00000008
    bne  5f
    tst  r4, #0x00000010
    bne  6f
    tst  r4, #0x00000020
    bne  7f
    tst  r4, #0x00000040
    bne  8f
    tst  r4, #0x00000080
    bne  9f

    add  r4, r4, #1
    b    10f

2:  sub  r4, r4, #3
    b    10f
3:  add  r4, r4, #7
    b    10f
4:  eor  r4, r4, #0x11
    b    10f
5:  add  r4, r4, r4, ror #3
    b    10f
6:  sub  r4, r4, r4, lsr #2
    b    10f
7:  eor  r4, r4, r4, lsr #1
    b    10f
8:  add  r4, r4, #0x123
    b    10f
9:  sub  r4, r4, #0x55

10:
    subs r0, r0, #1
    bne  1b

    mov  r0, r4
    pop  {r4, pc}

.size branch_bench_ram, .-branch_bench_ram
