#ifndef CHARLIE86_INSTRUCTIONS_H
#define CHARLIE86_INSTRUCTIONS_H

#define OPCODE_MOV_A1_MEMBYTE2AL 0xA0
#define OPCODE_MOV_A1_MEMWORD2AX 0xA1

#define OPCODE_MOV_B0_IMBYTE2AL 0xB0
#define OPCODE_MOV_B4_IMBYTE2AH 0xB4
#define OPCODE_MOV_B8_IMWORD2AX 0xB8

#define OPCODE_MOV_B3_IMBYTE2BL 0xB3
#define OPCODE_MOV_B7_IMBYTE2BH 0xB7
#define OPCODE_MOV_BB_IMWORD2BX 0xBB

#define OPCODE_MOV_B1_IMBYTE2CL 0xB1
#define OPCODE_MOV_B5_IMBYTE2CH 0xB5
#define OPCODE_MOV_B9_IMWORD2CX 0xB9

#define OPCODE_MOV_B2_IMBYTE2DL 0xB2
#define OPCODE_MOV_B6_IMBYTE2DH 0xB6
#define OPCODE_MOV_BA_IMWORD2DX 0xBA

#define OPCODE_MOV_BC_IMWORD2SP 0xBC
#define OPCODE_MOV_BD_IMWORD2BP 0xBD
#define OPCODE_MOV_BE_IMWORD2SI 0xBE
#define OPCODE_MOV_BF_IMWORD2DI 0xBF

#define OPCODE_MOV_8C_SR2RGWORD 0x8C
#define OPCODE_MOV_8E_RGWORD2SR 0x8E

#define OPCODE_MOV_89_GR2RGWORD 0x89
#define OPCODE_MOV_8B_RGWORD2GR 0x8B

#define OPCODE_MOV_C6_MRMOVBYTE1 0xC6
#define OPCODE_MOV_C7_MRMOVWORD1 0xC7

#define OPCODE_MOV_8A_MRMOVBYTE1 0x8A
#define OPCODE_MOV_88_MRMOVBYTE2 0x88

#define OPCODE_CMP_3C_BYTEINAL  0x3C
#define OPCODE_CMP_3D_WORDINAX  0x3D

#define OPCODE_JE_74_8REL       0x74
#define OPCODE_JNE_75_8REL      0x75

#define OPCODE_JC_72_8REL       0x72

#define OPCODE_JNC_73_8REL      0x73
#define OPCODE_JA_77_8REL       0x77
#define OPCODE_JNG_7E_8REL      0x7E
#define OPCODE_JG_7F_8REL       0x7F

#define OPCODE_JMP_REL16        0xE9
#define OPCODE_JMP_REL8         0xEB

#define OPCODE_CALL_E8_NEAR     0xE8

#define OPCODE_INT              0xCD
#define OPCODE_BREAKPOINT       0xCC

#define OPCODE_NOP              0x90

#define OPCODE_STI              0xFB
#define OPCODE_CLD              0xFC

#define OPCODE_CLI              0xFA
#define OPCODE_HLT              0xF4

#define OPCODE_LODSB            0xAC
#define OPCODE_LODSW            0xAD

#define OPCODE_OR_08_BYTE       0x08
#define OPCODE_OR_09_WORD       0x09

#define OPCODE_ADD_83_8IM       0x83
#define OPCODE_ADD_81_16IM      0x81

#define OPCODE_ADD_04_ALIM      0x04
#define OPCODE_ADD_05_AXIM      0x05
#define OPCODE_ADD_80_16IM      0x80

#define OPCODE_ADD_00_8RM       0x00
#define OPCODE_ADD_01_16RM      0x01

#define OPCODE_CMP_83_8IM       0x83
#define OPCODE_CMP_81_16IM      0x81
#define OPCODE_CMP_80_16IM      0x80

#define OPCODE_CMP_38_8D        0x38
#define OPCODE_CMP_39_16D       0x39

#define OPCODE_XOR_30_BYTE      0x30
#define OPCODE_XOR_31_WORD      0x31

#define OPCODE_MUL_F6_BYTE      0xF6
#define OPCODE_MUL_F7_WORD      0xF7

#define OPCODE_PUSH_AX_REG      0x50
#define OPCODE_PUSH_CX_REG      0x51
#define OPCODE_PUSH_DX_REG      0x52
#define OPCODE_PUSH_BX_REG      0x53

#define OPCODE_PUSH_SP_REG      0x54
#define OPCODE_PUSH_BP_REG      0x55
#define OPCODE_PUSH_SI_REG      0x56
#define OPCODE_PUSH_DI_REG      0x57

#define OPCODE_PUSH_CS_REG      0x0E
#define OPCODE_PUSH_DS_REG      0x1E
#define OPCODE_PUSH_SS_REG      0x16
#define OPCODE_PUSH_ES_REG      0x06

#define OPCODE_POP_DS_REG       0x1F
#define OPCODE_POP_SS_REG       0x17
#define OPCODE_POP_ES_REG       0x07

#define OPCODE_POP_AX_REG       0x58
#define OPCODE_POP_CX_REG       0x59
#define OPCODE_POP_DX_REG       0x5A
#define OPCODE_POP_BX_REG       0x5B

#define OPCODE_POP_SP_REG       0x5C
#define OPCODE_POP_BP_REG       0x5D
#define OPCODE_POP_SI_REG       0x5E
#define OPCODE_POP_DI_REG       0x5F

#define OPCODE_RET              0xC3

#define OPCODE_INC_FE_8MR       0xFE
#define OPCODE_INC_FF_16MR      0xFF

#define OPCODE_INC_AX_REG       0x40
#define OPCODE_INC_CX_REG       0x41
#define OPCODE_INC_DX_REG       0x42
#define OPCODE_INC_BX_REG       0x43

#define OPCODE_INC_SP_REG       0x44
#define OPCODE_INC_BP_REG       0x45
#define OPCODE_INC_SI_REG       0x46
#define OPCODE_INC_DI_REG       0x47

#define OPCODE_SUB_83_8IM       0x83

#define OPCODE_STOSB            0xAA
#define OPCODE_STOSW            0xAB

#define OPCODE_REP              0xF3
#define OPCODE_MOVSW            0xA5

#define OPCODE_JMP_EA_FAR       0xEA

#define OPCODE_OUT_AL           0xE6
#define OPCODE_IN_AL            0xE4

#define OPCODE_OUT_AX           0xE7
#define OPCODE_IN_AX            0xE5

#define OPCODE_AND_AL_IM8       0x24
#define OPCODE_AND_AX_IM16      0x25

#define OPCODE_LOOP_REL8        0xE2

#define OPCODE_PUSHA            0x60 // not strictly 8086, but handy
#define OPCODE_POPA             0x61 // ditto

#define OPCODE_SHR_D0_RM8       0xD0

#define OPCODE_TEST_AL          0xA8
#define OPCODE_SUB_29           0x29

#define OPCODE_XOR_AL_8IM       0x34

#define MOV_MEM_AX_16AM         0xA3

#define OPCODE_ADD_03           0x03

#define XCHG_AX_CX              0x91
#define XCHG_AX_DX              0x92
#define XCHG_AX_BX              0x93
#define XCHG_AX_SP              0x94
#define XCHG_AX_BP              0x95
#define XCHG_AX_SI              0x96
#define XCHG_AX_DI              0x97

#define OPCODE_SHRBYCL          0xD2

#define OPCODE_DEC_SI           0x4E
#define OPCODE_DEC_FE_MR        0xFE

#define OPCODE_CMPSB            0xA6

#define OPCODE_XOR_32_MR8       0x32
#define OPCODE_XOR_30_MR8_REV   0x30

#define OPCODE_XOR_33_MR16       0x33

#define OPCODE_STC              0xF9

#endif //CHARLIE86_INSTRUCTIONS_H
