//------------------------------------------------------------------------------
//
// Copyright (C) 2007-2008, Freescale Semiconductor, Inc. All Rights Reserved.
// THIS SOURCE CODE, AND ITS USE AND DISTRIBUTION, IS SUBJECT TO THE TERMS
// AND CONDITIONS OF THE APPLICABLE LICENSE AGREEMENT
//
//------------------------------------------------------------------------------
//
//
//  Header:  soc_macros.h
//
//  Provides common macro definitions SOC/BSP code development.
//------------------------------------------------------------------------------
#ifndef __SOCARM_MACROS_H__
#define __SOCARM_MACROS_H__

#define BASEREG    0
#define SETREG     1
#define CLRREG     2
#define TOGREG     3

// Bitfield macros that use rely on bitfield width/shift information
// defined in SOC header files
#define CSP_BITFMASK(bit) (((1U << (bit ## _WID)) - 1) << (bit ## _LSH))
#define CSP_BITFVAL(bit, val) ((val) << (bit ## _LSH))

// Undefine previous implementations of peripheral access macros since
// we want to "own" the definitions and avoid redefinition warnings
// resulting from source code that includes oal_io.h
#undef INREG8
#undef OUTREG8
#undef SETREG8
#undef CLRREG8
#undef INREG16
#undef OUTREG16
#undef SETREG16
#undef CLRREG16
#undef INREG32
#undef OUTREG32
#undef SETREG32
#undef CLRREG32

// Macros for accessing peripheral registers using DDK macros/functions
#define INREG8(x)           READ_REGISTER_UCHAR((UCHAR*)(x))
#define OUTREG8(x, y)       WRITE_REGISTER_UCHAR((UCHAR*)(x), (UCHAR)(y))
#define SETREG8(x, y)       OUTREG8(x, INREG8(x)|(y))
#define CLRREG8(x, y)       OUTREG8(x, INREG8(x)&~(y))
#define INSREG8(addr, mask, val) OUTREG8(addr, ((INREG8(addr)&(~(mask))) | val))
#define EXTREG8(addr, mask, lsh) ((INREG8(addr) & mask) >> lsh)

#define INREG16(x)          READ_REGISTER_USHORT((USHORT*)(x))
#define OUTREG16(x, y)      WRITE_REGISTER_USHORT((USHORT*)(x),(USHORT)(y))
#define SETREG16(x, y)      OUTREG16(x, INREG16(x)|(y))
#define CLRREG16(x, y)      OUTREG16(x, INREG16(x)&~(y))
#define INSREG16(addr, mask, val) OUTREG16(addr, ((INREG16(addr)&(~(mask))) | val))
#define EXTREG16(addr, mask, lsh) ((INREG16(addr) & mask) >> lsh)

#define INREG32(x)          READ_REGISTER_ULONG((ULONG*)(x))
#define OUTREG32(x, y)      WRITE_REGISTER_ULONG((ULONG*)(x), (ULONG)(y))
#define SETREG32(x, y)      OUTREG32(x, INREG32(x)|(y))
#define CLRREG32(x, y)      OUTREG32(x, INREG32(x)&~(y))
#define INSREG32(addr, mask, val) OUTREG32(addr, ((INREG32(addr)&(~(mask))) | val))
#define EXTREG32(addr, mask, lsh) ((INREG32(addr) & mask) >> lsh)


//STMP macros for single instance registers
#define BF_SET(reg, field)       HW_ ## reg ## _SET(BM_ ## reg ## _ ## field)
#define BF_CLR(reg, field)       HW_ ## reg ## _CLR(BM_ ## reg ## _ ## field)
#define BF_TOG(reg, field)       HW_ ## reg ## _TOG(BM_ ## reg ## _ ## field)

#define BF_SETV(reg, field, v)   HW_ ## reg ## _SET(BF_ ## reg ## _ ## field(v))
#define BF_CLRV(reg, field, v)   HW_ ## reg ## _CLR(BF_ ## reg ## _ ## field(v))
#define BF_TOGV(reg, field, v)   HW_ ## reg ## _TOG(BF_ ## reg ## _ ## field(v))

#define BV_FLD(reg, field, sym)  BF_ ## reg ## _ ## field(BV_ ## reg ## _ ## field ## __ ## sym)
#define BV_VAL(reg, field, sym)  BV_ ## reg ## _ ## field ## __ ## sym

#define BF_RD(reg, field)        HW_ ## reg.B.field
#define BF_WR(reg, field, v)     BW_ ## reg ## _ ## field(v)

#define BF_CS1(reg, f1, v1)  \
    (HW_ ## reg ## _CLR(BM_ ## reg ## _ ## f1),      \
     HW_ ## reg ## _SET(BF_ ## reg ## _ ## f1(v1)))

#define BF_CS2(reg, f1, v1, f2, v2)  \
    (HW_ ## reg ## _CLR(BM_ ## reg ## _ ## f1 |      \
                        BM_ ## reg ## _ ## f2),      \
     HW_ ## reg ## _SET(BF_ ## reg ## _ ## f1(v1) |  \
                        BF_ ## reg ## _ ## f2(v2)))

#define BF_CS3(reg, f1, v1, f2, v2, f3, v3)  \
    (HW_ ## reg ## _CLR(BM_ ## reg ## _ ## f1 |      \
                        BM_ ## reg ## _ ## f2 |      \
                        BM_ ## reg ## _ ## f3),      \
     HW_ ## reg ## _SET(BF_ ## reg ## _ ## f1(v1) |  \
                        BF_ ## reg ## _ ## f2(v2) |  \
                        BF_ ## reg ## _ ## f3(v3)))

#define BF_CS4(reg, f1, v1, f2, v2, f3, v3, f4, v4)  \
    (HW_ ## reg ## _CLR(BM_ ## reg ## _ ## f1 |      \
                        BM_ ## reg ## _ ## f2 |      \
                        BM_ ## reg ## _ ## f3 |      \
                        BM_ ## reg ## _ ## f4),      \
     HW_ ## reg ## _SET(BF_ ## reg ## _ ## f1(v1) |  \
                        BF_ ## reg ## _ ## f2(v2) |  \
                        BF_ ## reg ## _ ## f3(v3) |  \
                        BF_ ## reg ## _ ## f4(v4)))

#define BF_CS5(reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5)  \
    (HW_ ## reg ## _CLR(BM_ ## reg ## _ ## f1 |      \
                        BM_ ## reg ## _ ## f2 |      \
                        BM_ ## reg ## _ ## f3 |      \
                        BM_ ## reg ## _ ## f4 |      \
                        BM_ ## reg ## _ ## f5),      \
     HW_ ## reg ## _SET(BF_ ## reg ## _ ## f1(v1) |  \
                        BF_ ## reg ## _ ## f2(v2) |  \
                        BF_ ## reg ## _ ## f3(v3) |  \
                        BF_ ## reg ## _ ## f4(v4) |  \
                        BF_ ## reg ## _ ## f5(v5)))

#define BF_CS6(reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6)  \
    (HW_ ## reg ## _CLR(BM_ ## reg ## _ ## f1 |      \
                        BM_ ## reg ## _ ## f2 |      \
                        BM_ ## reg ## _ ## f3 |      \
                        BM_ ## reg ## _ ## f4 |      \
                        BM_ ## reg ## _ ## f5 |      \
                        BM_ ## reg ## _ ## f6),      \
     HW_ ## reg ## _SET(BF_ ## reg ## _ ## f1(v1) |  \
                        BF_ ## reg ## _ ## f2(v2) |  \
                        BF_ ## reg ## _ ## f3(v3) |  \
                        BF_ ## reg ## _ ## f4(v4) |  \
                        BF_ ## reg ## _ ## f5(v5) |  \
                        BF_ ## reg ## _ ## f6(v6)))

#define BF_CS7(reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7)  \
    (HW_ ## reg ## _CLR(BM_ ## reg ## _ ## f1 |      \
                        BM_ ## reg ## _ ## f2 |      \
                        BM_ ## reg ## _ ## f3 |      \
                        BM_ ## reg ## _ ## f4 |      \
                        BM_ ## reg ## _ ## f5 |      \
                        BM_ ## reg ## _ ## f6 |      \
                        BM_ ## reg ## _ ## f7),      \
     HW_ ## reg ## _SET(BF_ ## reg ## _ ## f1(v1) |  \
                        BF_ ## reg ## _ ## f2(v2) |  \
                        BF_ ## reg ## _ ## f3(v3) |  \
                        BF_ ## reg ## _ ## f4(v4) |  \
                        BF_ ## reg ## _ ## f5(v5) |  \
                        BF_ ## reg ## _ ## f6(v6) |  \
                        BF_ ## reg ## _ ## f7(v7)))

#define BF_CS8(reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7, f8, v8)  \
    (HW_ ## reg ## _CLR(BM_ ## reg ## _ ## f1 |      \
                        BM_ ## reg ## _ ## f2 |      \
                        BM_ ## reg ## _ ## f3 |      \
                        BM_ ## reg ## _ ## f4 |      \
                        BM_ ## reg ## _ ## f5 |      \
                        BM_ ## reg ## _ ## f6 |      \
                        BM_ ## reg ## _ ## f7 |      \
                        BM_ ## reg ## _ ## f8),      \
     HW_ ## reg ## _SET(BF_ ## reg ## _ ## f1(v1) |  \
                        BF_ ## reg ## _ ## f2(v2) |  \
                        BF_ ## reg ## _ ## f3(v3) |  \
                        BF_ ## reg ## _ ## f4(v4) |  \
                        BF_ ## reg ## _ ## f5(v5) |  \
                        BF_ ## reg ## _ ## f6(v6) |  \
                        BF_ ## reg ## _ ## f7(v7) |  \
                        BF_ ## reg ## _ ## f8(v8)))


//
// macros for multiple instance registers
//

#define BF_SETn(reg, n, field)       HW_ ## reg ## _SET(n, BM_ ## reg ## _ ## field)
#define BF_CLRn(reg, n, field)       HW_ ## reg ## _CLR(n, BM_ ## reg ## _ ## field)
#define BF_TOGn(reg, n, field)       HW_ ## reg ## _TOG(n, BM_ ## reg ## _ ## field)

#define BF_SETVn(reg, n, field, v)   HW_ ## reg ## _SET(n, BF_ ## reg ## _ ## field(v))
#define BF_CLRVn(reg, n, field, v)   HW_ ## reg ## _CLR(n, BF_ ## reg ## _ ## field(v))
#define BF_TOGVn(reg, n, field, v)   HW_ ## reg ## _TOG(n, BF_ ## reg ## _ ## field(v))

#define BV_FLDn(reg, n, field, sym)  BF_ ## reg ## _ ## field(BV_ ## reg ## _ ## field ## __ ## sym)
#define BV_VALn(reg, n, field, sym)  BV_ ## reg ## _ ## field ## __ ## sym

#define BF_RDn(reg, n, field)        HW_ ## reg(n).B.field
#define BF_WRn(reg, n, field, v)     BW_ ## reg ## _ ## field(n, v)

#define BF_CS1n(reg, n, f1, v1)  \
    (HW_ ## reg ## _CLR(n, (BM_ ## reg ## _ ## f1)),      \
     HW_ ## reg ## _SET(n, (BF_ ## reg ## _ ## f1(v1))))

#define BF_CS2n(reg, n, f1, v1, f2, v2)  \
    (HW_ ## reg ## _CLR(n, (BM_ ## reg ## _ ## f1 |       \
                            BM_ ## reg ## _ ## f2)),      \
     HW_ ## reg ## _SET(n, (BF_ ## reg ## _ ## f1(v1) |   \
                            BF_ ## reg ## _ ## f2(v2))))

#define BF_CS3n(reg, n, f1, v1, f2, v2, f3, v3)  \
    (HW_ ## reg ## _CLR(n, (BM_ ## reg ## _ ## f1 |       \
                            BM_ ## reg ## _ ## f2 |       \
                            BM_ ## reg ## _ ## f3)),      \
     HW_ ## reg ## _SET(n, (BF_ ## reg ## _ ## f1(v1) |   \
                            BF_ ## reg ## _ ## f2(v2) |   \
                            BF_ ## reg ## _ ## f3(v3))))

#define BF_CS4n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4)  \
    (HW_ ## reg ## _CLR(n, (BM_ ## reg ## _ ## f1 |       \
                            BM_ ## reg ## _ ## f2 |       \
                            BM_ ## reg ## _ ## f3 |       \
                            BM_ ## reg ## _ ## f4)),      \
     HW_ ## reg ## _SET(n, (BF_ ## reg ## _ ## f1(v1) |   \
                            BF_ ## reg ## _ ## f2(v2) |   \
                            BF_ ## reg ## _ ## f3(v3) |   \
                            BF_ ## reg ## _ ## f4(v4))))

#define BF_CS5n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5)  \
    (HW_ ## reg ## _CLR(n, (BM_ ## reg ## _ ## f1 |       \
                            BM_ ## reg ## _ ## f2 |       \
                            BM_ ## reg ## _ ## f3 |       \
                            BM_ ## reg ## _ ## f4 |       \
                            BM_ ## reg ## _ ## f5)),      \
     HW_ ## reg ## _SET(n, (BF_ ## reg ## _ ## f1(v1) |   \
                            BF_ ## reg ## _ ## f2(v2) |   \
                            BF_ ## reg ## _ ## f3(v3) |   \
                            BF_ ## reg ## _ ## f4(v4) |   \
                            BF_ ## reg ## _ ## f5(v5))))

#define BF_CS6n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6)  \
    (HW_ ## reg ## _CLR(n, (BM_ ## reg ## _ ## f1 |       \
                            BM_ ## reg ## _ ## f2 |       \
                            BM_ ## reg ## _ ## f3 |       \
                            BM_ ## reg ## _ ## f4 |       \
                            BM_ ## reg ## _ ## f5 |       \
                            BM_ ## reg ## _ ## f6)),      \
     HW_ ## reg ## _SET(n, (BF_ ## reg ## _ ## f1(v1) |   \
                            BF_ ## reg ## _ ## f2(v2) |   \
                            BF_ ## reg ## _ ## f3(v3) |   \
                            BF_ ## reg ## _ ## f4(v4) |   \
                            BF_ ## reg ## _ ## f5(v5) |   \
                            BF_ ## reg ## _ ## f6(v6))))

#define BF_CS7n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7)  \
    (HW_ ## reg ## _CLR(n, (BM_ ## reg ## _ ## f1 |       \
                            BM_ ## reg ## _ ## f2 |       \
                            BM_ ## reg ## _ ## f3 |       \
                            BM_ ## reg ## _ ## f4 |       \
                            BM_ ## reg ## _ ## f5 |       \
                            BM_ ## reg ## _ ## f6 |       \
                            BM_ ## reg ## _ ## f7)),      \
     HW_ ## reg ## _SET(n, (BF_ ## reg ## _ ## f1(v1) |   \
                            BF_ ## reg ## _ ## f2(v2) |   \
                            BF_ ## reg ## _ ## f3(v3) |   \
                            BF_ ## reg ## _ ## f4(v4) |   \
                            BF_ ## reg ## _ ## f5(v5) |   \
                            BF_ ## reg ## _ ## f6(v6) |   \
                            BF_ ## reg ## _ ## f7(v7))))

#define BF_CS8n(reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7, f8, v8)  \
    (HW_ ## reg ## _CLR(n, (BM_ ## reg ## _ ## f1 |       \
                            BM_ ## reg ## _ ## f2 |       \
                            BM_ ## reg ## _ ## f3 |       \
                            BM_ ## reg ## _ ## f4 |       \
                            BM_ ## reg ## _ ## f5 |       \
                            BM_ ## reg ## _ ## f6 |       \
                            BM_ ## reg ## _ ## f7 |       \
                            BM_ ## reg ## _ ## f8)),      \
     HW_ ## reg ## _SET(n, (BF_ ## reg ## _ ## f1(v1) |   \
                            BF_ ## reg ## _ ## f2(v2) |   \
                            BF_ ## reg ## _ ## f3(v3) |   \
                            BF_ ## reg ## _ ## f4(v4) |   \
                            BF_ ## reg ## _ ## f5(v5) |   \
                            BF_ ## reg ## _ ## f6(v6) |   \
                            BF_ ## reg ## _ ## f7(v7) |   \
                            BF_ ## reg ## _ ## f8(v8))))

///////

//
// macros for single instance registers
//

#define BFi_SET(i, reg, field)       HWi_ ## reg ## _SET(i, BM_ ## reg ## _ ## field)
#define BFi_CLR(i, reg, field)       HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## field)
#define BFi_TOG(i, reg, field)       HWi_ ## reg ## _TOG(i, BM_ ## reg ## _ ## field)

#define BFi_SETV(i, reg, field, v)   HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## field(v))
#define BFi_CLRV(i, reg, field, v)   HWi_ ## reg ## _CLR(i, BF_ ## reg ## _ ## field(v))
#define BFi_TOGV(i, reg, field, v)   HWi_ ## reg ## _TOG(i, BF_ ## reg ## _ ## field(v))

#define BFi_RD(i, reg, field)        HWi_ ## reg(i).B.field
#define BFi_WR(i, reg, field, v)     BWi_ ## reg ## _ ## field(i, v)

#define BFi_CS1(i, reg, f1, v1)  \
    (HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## f1),      \
     HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## f1(v1)))

#define BFi_CS2(i, reg, f1, v1, f2, v2)  \
    (HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## f1 |      \
                         BM_ ## reg ## _ ## f2),      \
     HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## f1(v1) |  \
                         BF_ ## reg ## _ ## f2(v2)))

#define BFi_CS3(i, reg, f1, v1, f2, v2, f3, v3)  \
    (HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## f1 |      \
                         BM_ ## reg ## _ ## f2 |      \
                         BM_ ## reg ## _ ## f3),      \
     HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## f1(v1) |  \
                         BF_ ## reg ## _ ## f2(v2) |  \
                         BF_ ## reg ## _ ## f3(v3)))

#define BFi_CS4(i, reg, f1, v1, f2, v2, f3, v3, f4, v4)  \
    (HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## f1 |      \
                         BM_ ## reg ## _ ## f2 |      \
                         BM_ ## reg ## _ ## f3 |      \
                         BM_ ## reg ## _ ## f4),      \
     HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## f1(v1) |  \
                         BF_ ## reg ## _ ## f2(v2) |  \
                         BF_ ## reg ## _ ## f3(v3) |  \
                         BF_ ## reg ## _ ## f4(v4)))

#define BFi_CS5(i, reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5)  \
    (HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## f1 |      \
                         BM_ ## reg ## _ ## f2 |      \
                         BM_ ## reg ## _ ## f3 |      \
                         BM_ ## reg ## _ ## f4 |      \
                         BM_ ## reg ## _ ## f5),      \
     HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## f1(v1) |  \
                         BF_ ## reg ## _ ## f2(v2) |  \
                         BF_ ## reg ## _ ## f3(v3) |  \
                         BF_ ## reg ## _ ## f4(v4) |  \
                         BF_ ## reg ## _ ## f5(v5)))

#define BFi_CS6(i, reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6)  \
    (HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## f1 |      \
                         BM_ ## reg ## _ ## f2 |      \
                         BM_ ## reg ## _ ## f3 |      \
                         BM_ ## reg ## _ ## f4 |      \
                         BM_ ## reg ## _ ## f5 |      \
                         BM_ ## reg ## _ ## f6),      \
     HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## f1(v1) |  \
                         BF_ ## reg ## _ ## f2(v2) |  \
                         BF_ ## reg ## _ ## f3(v3) |  \
                         BF_ ## reg ## _ ## f4(v4) |  \
                         BF_ ## reg ## _ ## f5(v5) |  \
                         BF_ ## reg ## _ ## f6(v6)))

#define BFi_CS7(i, reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7)  \
    (HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## f1 |      \
                         BM_ ## reg ## _ ## f2 |      \
                         BM_ ## reg ## _ ## f3 |      \
                         BM_ ## reg ## _ ## f4 |      \
                         BM_ ## reg ## _ ## f5 |      \
                         BM_ ## reg ## _ ## f6 |      \
                         BM_ ## reg ## _ ## f7),      \
     HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## f1(v1) |  \
                         BF_ ## reg ## _ ## f2(v2) |  \
                         BF_ ## reg ## _ ## f3(v3) |  \
                         BF_ ## reg ## _ ## f4(v4) |  \
                         BF_ ## reg ## _ ## f5(v5) |  \
                         BF_ ## reg ## _ ## f6(v6) |  \
                         BF_ ## reg ## _ ## f7(v7)))

#define BFi_CS8(i, reg, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7, f8, v8)  \
    (HWi_ ## reg ## _CLR(i, BM_ ## reg ## _ ## f1 |      \
                         BM_ ## reg ## _ ## f2 |      \
                         BM_ ## reg ## _ ## f3 |      \
                         BM_ ## reg ## _ ## f4 |      \
                         BM_ ## reg ## _ ## f5 |      \
                         BM_ ## reg ## _ ## f6 |      \
                         BM_ ## reg ## _ ## f7 |      \
                         BM_ ## reg ## _ ## f8),      \
     HWi_ ## reg ## _SET(i, BF_ ## reg ## _ ## f1(v1) |  \
                         BF_ ## reg ## _ ## f2(v2) |  \
                         BF_ ## reg ## _ ## f3(v3) |  \
                         BF_ ## reg ## _ ## f4(v4) |  \
                         BF_ ## reg ## _ ## f5(v5) |  \
                         BF_ ## reg ## _ ## f6(v6) |  \
                         BF_ ## reg ## _ ## f7(v7) |  \
                         BF_ ## reg ## _ ## f8(v8)))


//
// macros for multiple instance registers
//

#define BFi_SETn(i, reg, n, field)       HWi_ ## reg ## _SET(i, n, BM_ ## reg ## _ ## field)
#define BFi_CLRn(i, reg, n, field)       HWi_ ## reg ## _CLR(i, n, BM_ ## reg ## _ ## field)
#define BFi_TOGn(i, reg, n, field)       HWi_ ## reg ## _TOG(i, n, BM_ ## reg ## _ ## field)

#define BFi_SETVn(i, reg, n, field, v)   HWi_ ## reg ## _SET(i, n, BF_ ## reg ## _ ## field(v))
#define BFi_CLRVn(i, reg, n, field, v)   HWi_ ## reg ## _CLR(i, n, BF_ ## reg ## _ ## field(v))
#define BFi_TOGVn(i, reg, n, field, v)   HWi_ ## reg ## _TOG(i, n, BF_ ## reg ## _ ## field(v))

#define BFi_RDn(i, reg, n, field)        HWi_ ## reg(i, n).B.field
#define BFi_WRn(i, reg, n, field, v)     BWi_ ## reg ## _ ## field(i, n, v)

#define BFi_CS1n(i, reg, n, f1, v1)  \
    (HWi_ ## reg ## _CLR(i, n, (BM_ ## reg ## _ ## f1)),      \
     HWi_ ## reg ## _SET(i, n, (BF_ ## reg ## _ ## f1(v1))))

#define BFi_CS2n(i, reg, n, f1, v1, f2, v2)  \
    (HWi_ ## reg ## _CLR(i, n, (BM_ ## reg ## _ ## f1 |       \
                                BM_ ## reg ## _ ## f2)),      \
     HWi_ ## reg ## _SET(i, n, (BF_ ## reg ## _ ## f1(v1) |   \
                                BF_ ## reg ## _ ## f2(v2))))

#define BFi_CS3n(i, reg, n, f1, v1, f2, v2, f3, v3)  \
    (HWi_ ## reg ## _CLR(i, n, (BM_ ## reg ## _ ## f1 |       \
                                BM_ ## reg ## _ ## f2 |       \
                                BM_ ## reg ## _ ## f3)),      \
     HWi_ ## reg ## _SET(i, n, (BF_ ## reg ## _ ## f1(v1) |   \
                                BF_ ## reg ## _ ## f2(v2) |   \
                                BF_ ## reg ## _ ## f3(v3))))

#define BFi_CS4n(i, reg, n, f1, v1, f2, v2, f3, v3, f4, v4)  \
    (HWi_ ## reg ## _CLR(i, n, (BM_ ## reg ## _ ## f1 |       \
                                BM_ ## reg ## _ ## f2 |       \
                                BM_ ## reg ## _ ## f3 |       \
                                BM_ ## reg ## _ ## f4)),      \
     HWi_ ## reg ## _SET(i, n, (BF_ ## reg ## _ ## f1(v1) |   \
                                BF_ ## reg ## _ ## f2(v2) |   \
                                BF_ ## reg ## _ ## f3(v3) |   \
                                BF_ ## reg ## _ ## f4(v4))))

#define BFi_CS5n(i, reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5)  \
    (HWi_ ## reg ## _CLR(i, n, (BM_ ## reg ## _ ## f1 |       \
                                BM_ ## reg ## _ ## f2 |       \
                                BM_ ## reg ## _ ## f3 |       \
                                BM_ ## reg ## _ ## f4 |       \
                                BM_ ## reg ## _ ## f5)),      \
     HWi_ ## reg ## _SET(i, n, (BF_ ## reg ## _ ## f1(v1) |   \
                                BF_ ## reg ## _ ## f2(v2) |   \
                                BF_ ## reg ## _ ## f3(v3) |   \
                                BF_ ## reg ## _ ## f4(v4) |   \
                                BF_ ## reg ## _ ## f5(v5))))

#define BFi_CS6n(i, reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6)  \
    (HWi_ ## reg ## _CLR(i, n, (BM_ ## reg ## _ ## f1 |       \
                                BM_ ## reg ## _ ## f2 |       \
                                BM_ ## reg ## _ ## f3 |       \
                                BM_ ## reg ## _ ## f4 |       \
                                BM_ ## reg ## _ ## f5 |       \
                                BM_ ## reg ## _ ## f6)),      \
     HWi_ ## reg ## _SET(i, n, (BF_ ## reg ## _ ## f1(v1) |   \
                                BF_ ## reg ## _ ## f2(v2) |   \
                                BF_ ## reg ## _ ## f3(v3) |   \
                                BF_ ## reg ## _ ## f4(v4) |   \
                                BF_ ## reg ## _ ## f5(v5) |   \
                                BF_ ## reg ## _ ## f6(v6))))

#define BFi_CS7n(i, reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7)  \
    (HWi_ ## reg ## _CLR(i, n, (BM_ ## reg ## _ ## f1 |       \
                                BM_ ## reg ## _ ## f2 |       \
                                BM_ ## reg ## _ ## f3 |       \
                                BM_ ## reg ## _ ## f4 |       \
                                BM_ ## reg ## _ ## f5 |       \
                                BM_ ## reg ## _ ## f6 |       \
                                BM_ ## reg ## _ ## f7)),      \
     HWi_ ## reg ## _SET(i, n, (BF_ ## reg ## _ ## f1(v1) |   \
                                BF_ ## reg ## _ ## f2(v2) |   \
                                BF_ ## reg ## _ ## f3(v3) |   \
                                BF_ ## reg ## _ ## f4(v4) |   \
                                BF_ ## reg ## _ ## f5(v5) |   \
                                BF_ ## reg ## _ ## f6(v6) |   \
                                BF_ ## reg ## _ ## f7(v7))))

#define BFi_CS8n(i, reg, n, f1, v1, f2, v2, f3, v3, f4, v4, f5, v5, f6, v6, f7, v7, f8, v8)  \
    (HWi_ ## reg ## _CLR(i, n, (BM_ ## reg ## _ ## f1 |       \
                                BM_ ## reg ## _ ## f2 |       \
                                BM_ ## reg ## _ ## f3 |       \
                                BM_ ## reg ## _ ## f4 |       \
                                BM_ ## reg ## _ ## f5 |       \
                                BM_ ## reg ## _ ## f6 |       \
                                BM_ ## reg ## _ ## f7 |       \
                                BM_ ## reg ## _ ## f8)),      \
     HWi_ ## reg ## _SET(i, n, (BF_ ## reg ## _ ## f1(v1) |   \
                                BF_ ## reg ## _ ## f2(v2) |   \
                                BF_ ## reg ## _ ## f3(v3) |   \
                                BF_ ## reg ## _ ## f4(v4) |   \
                                BF_ ## reg ## _ ## f5(v5) |   \
                                BF_ ## reg ## _ ## f6(v6) |   \
                                BF_ ## reg ## _ ## f7(v7) |   \
                                BF_ ## reg ## _ ## f8(v8))))

// Macros to create Unicode function name
#define WIDEN2(x) L ## x
#define WIDEN(x) WIDEN2(x)
#define __WFUNCTION__ WIDEN(__FUNCTION__)


#endif // __SOCARM_MACROS_H__

