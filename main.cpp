#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>
#include <stack>

#include "common.h"
#include "regs.h"
#include "mr.h"
#include "rm.h"
#include "video.h"
#include "flags.h"
#include "instructions.h"

//#define SILENT_EXECUTION // for a major speed boost
//#define NOREGDUMP // for a minor speed boost when simming long things

// undefine to disable, 16/17 is pretty fast but not too fast.
#define INSN_DELAY 16 // lol look below this basically has no unit
//#define INSN_DELAY_MS 2 // for >1ms insn delays, overrides the above.

//#define SINGLE_STEP // wait for keypress for every insn

#define BOOT_DISK "../disk/MSDOS33_1.img"

unsigned long long cycles = 0;

uint16_t lastcmp;

uint8_t SEG_OVERRIDE = REGISTER_DS;

callstack_t callstack;

char lastinsn[512];
char lastinsn2[512];
char lastinsn3[512];
char lastinsn4[512];

void rotatelog() {
    strcpy(lastinsn4, lastinsn3);
    strcpy(lastinsn3, lastinsn2);
    strcpy(lastinsn2, lastinsn);
}

void fixilog() {
    for (char & i : lastinsn) {
        if (i == '\n') {
            i = 0x20;
        }
    }
    for (char & i : lastinsn2) {
        if (i == '\n') {
            i = 0x20;
        }
    }
    for (char & i : lastinsn3) {
        if (i == '\n') {
            i = 0x20;
        }
    }
    for (char & i : lastinsn4) {
        if (i == '\n') {
            i = 0x20;
        }
    }
}

#ifndef SILENT_EXECUTION
#define FN_ILOG(...) printf(__VA_ARGS__); rotatelog(); snprintf(lastinsn,512,__VA_ARGS__); fixilog();
#else
#define FN_ILOG(...) rotatelog(); snprintf(lastinsn,512,__VA_ARGS__); fixilog();
#define NOREGDUMP
#endif

#define FN_ELOG(...) regdump_force(); rotatelog(); printf(__VA_ARGS__); snprintf(lastinsn,512,__VA_ARGS__); fixilog();

#define VIRTUAL_MEMORY_SIZE (1024 * 1024) // 1MiB of virtual memory for now

static uint8_t* virt_memory = NULL;

static uint16_t IP = 0;
static uint16_t regs[12]; // AX,BX,CX,DX SP,BP,SI,DI CS,DS,ES,SS

static int running = 1; // if the CPU is running, i.e executing instructions

static uint16_t FLAGS = 0;

uint8_t * getmemory() { return virt_memory; }
uint16_t getip() { return IP; }
uint16_t* getregs() { return regs; }
uint16_t getflags() { return FLAGS; }
int getrunning() { return running; }
char* getlastinsn() { return lastinsn; }
char* getlastinsn2() { return lastinsn2; }
char* getlastinsn3() { return lastinsn3; }
char* getlastinsn4() { return lastinsn4; }
uint32_t segcalc(uint16_t seg, uint16_t off);
uint32_t getstackptr() {
    return segcalc(REGISTER_SS, getregval(REGISTER_SP));
}
callstack_t getcallstack(){
    return callstack;
}
void loadsector(uint16_t sid, uint32_t destination, uint16_t count, const char* file);
unsigned long long getcycles(){
    return cycles;
}
void regdump_force();

uint16_t getregval(uint8_t regid);
void setregval(uint8_t regid, uint16_t val);

void clearflag(uint16_t FLAGID);
void setflag(uint16_t FLAGID);

void writeByte(uint8_t seg, uint16_t off, uint8_t val);

uint16_t dispoff = 0;

void interrupt(uint8_t intid) {
    if (!(FLAGS & FLAG_INT))
        return;

    if (intid == 0x10) { // BIOS display services
        if (getregval(REGISTER_AH) == 0x0E) {
            if (getregval(REGISTER_AL) >= 0x20) {
                virt_memory[0xB8000 + dispoff] = getregval(REGISTER_AL);
                //printf("VM BIOS print char: %c off=%d\n", getregval(REGISTER_AL), dispoff);
            } else if (getregval(REGISTER_AL) == '\n') {
                //printf("VM BIOS print NL off=%d\n", dispoff);
                dispoff+=80;
                dispoff -= (dispoff%80) + 1;
            } else if (getregval(REGISTER_AL) == '\r') {
                //printf("VM BIOS print CR off=%d\n", dispoff);
                dispoff -= (dispoff%80) + 1;
            }

            dispoff++;
        } else if (getregval(REGISTER_AH) == 0) {
            printf("VM change video mode to %d\n",getregval(REGISTER_AL));
            switch (getregval(REGISTER_AL)) {
                case 0x13:
                    initvideo(320,200,0xA0000, GRAPHICS_MODE_8BPP);
                    break;
                case 0x02:
                    initvideo(80,25,0xB8000, TEXT_MODE_16BPC);
                    break;
                case 0x03:
                    initvideo(80,25,0xB8000, TEXT_MODE_8BPC);
                    break;
                default:
                    printf("WARNING: Unimplemented video mode %d, defaulting to 03h(80x25 colour)\n",getregval(REGISTER_AH));
                    initvideo(80,25,0xB8000, TEXT_MODE_8BPC);
            }
        }
    } else if (intid == 0xFF) { // 0:0 jump callback
        printf("0000:0000 has been jumped to. Halting.\n");
    } else if (intid == 0xFE) { // sooper secret prints for in-VM things like the boot ROM
        printf("VM: %s\n", virt_memory + getregval(REGISTER_SI));
        setregval(REGISTER_SI, 0);
    } else if (intid == 0xFD) {
        printf("VM sleep %dms\n",getregval(REGISTER_AX));
        std::this_thread::sleep_for(std::chrono::milliseconds(getregval(REGISTER_AX)));
    } else if (intid == 0xFC) {
        printf("VM clear text display\n");
        dispoff = 0;
        memset(virt_memory + 0xB8000, 0, 80*25);
    } else if (intid == 0x1A) {
        setregval(REGISTER_DX, 0);
    } else if (intid == 0x16) {
        if (getregval(REGISTER_AH) == 0x01) {
            if (isKeyPressed()) {
                setflag(FLAG_ZERO);
            } else {
                clearflag(FLAG_ZERO);
            }
        } else if (getregval(REGISTER_AH) == 0x00) { // wait for and return key
            waitForKey();
            setregval(REGISTER_AL, getLatestKey());
            setregval(REGISTER_AH, getLatestKey());
        }
    } else if (intid == 0x13) {
        uint8_t AH = getregval(REGISTER_AH);
        printf("VM Disk system interrupt, AH=%02X\n",AH);
        if (AH==0) {
            printf("VM Disk system reset. (NOP)\n");
        } else if (AH==2) {
            uint8_t sector = getregval(REGISTER_CL);
            uint8_t head = getregval(REGISTER_DH);
            uint8_t cylinder = getregval(REGISTER_CH);

            uint8_t amt = getregval(REGISTER_AL);

            uint32_t LBA = (cylinder * 1 * 8) + (head * 8) + (sector/* - 1*/);

            uint16_t ES = getregval(REGISTER_ES);
            uint16_t BX = getregval(REGISTER_BX);

            regdump_force();
            printf("VM read sector, from %d to %d [LBA=%08X, C=%d, H=%d, S=%d, bufstart = %04X:%04X (%08X)]\n",
                   sector, sector + amt, LBA, cylinder, head, sector, ES, BX, segcalc(REGISTER_ES, BX));

            loadsector(LBA, segcalc(REGISTER_ES, BX), amt, BOOT_DISK);
        } else {
            printf("VM illegal disk interrupt\n");
            running = false;
        }
    } else {
        printf("VM illegal interrupt %02X AH=%02X\n", intid,getregval(REGISTER_AH));
        running = false;
    }
}

void setregval(uint8_t regid, uint16_t val) {
    if (regid < 4) {
        //printf("SET LOW REG %s[%s] TO %x\n",getregname(regid), getregname(regid+8), val&0x00FF);
        regs[regid] &= 0xFF00;
        regs[regid] |= val & 0x00FF;
    } else if (regid < 8) {
        //printf("SET HIGH REG %s[%s] TO %02x\n",getregname(regid), getregname(regid+4), (val&0xFF));
        regs[regid - 4] &= 0x00FF;
        regs[regid - 4] |= (val & 0xFF) << 8;
    } else {
        //printf("SET REG %s[%d] TO %x\n",getregname(regid), regid - 8, val);
        regs[regid - 8] = val;
    }
}

uint16_t getregval(uint8_t regid) {
    if (regid < 4) {
        return regs[regid] & 0xFF;
    } else if (regid < 8) {
        return (regs[regid - 4] & 0xFF00) >> 8;
    } else {
        return regs[regid - 8];
    }
}

void regdump_force() {
    printf("[");
    for (int i=0;i<0x8;i++) {
        printf("%s=%04X ", getregname(i + 8), getregval(i + 8));
    }
    printf("IP=%04X CS=%04X DS=%04X ES=%04X SS=%04X FLAGS=%04X] ",IP,
           getregval(REGISTER_CS), getregval(SEG_OVERRIDE), getregval(REGISTER_ES), getregval(REGISTER_SS), FLAGS);
}

void regdump() {
#ifndef NOREGDUMP
    regdump_force();
#endif
}

uint8_t SEG = REGISTER_CS;

void loadsector(uint16_t sid, uint32_t destination, uint16_t count, const char* file) {
    std::ifstream input( file, std::ios::binary );
    std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(input), {});
    if (buffer.size() / 512 < count || buffer.size() < (sid*512 + count*512)) {
        fprintf(stderr, "Warning: file %s doesn't contain enough data to read %d sectors (fz=%d, rq=%d). Aborting.\n", file, count, buffer.size(), (sid*512 + count*512));
        input.close();
        return;
    }
    memcpy(virt_memory + destination, buffer.data() + (sid*512), count * 512);
    printf("Loaded %d sectors from %s [sector %d] to 0x%X\n", count, file, sid, destination);
}

void call_xhandler(uint16_t s, uint16_t a, uint16_t rip) {
    std::pair<uint16_t, uint16_t> p(getregval(SEG), rip);
    std::pair<uint16_t, uint16_t> p2(s,a);
    callstack.push(CALLSTACK_ENTRY_INIT(p, p2));
}

void ret_xhandler(uint16_t s, uint16_t a) {
    callstack.pop();
}

void jump(uint16_t addr) {
    IP = addr;
}

uint32_t segcalc(uint16_t seg, uint16_t off) {
    uint32_t ret = seg;
    ret = ret << 4;
    ret += off;
    return ret;
}

uint32_t pushw(uint16_t val) {
    uint32_t stackptr = segcalc(getregval(REGISTER_SS), getregval(REGISTER_SP));
    setregval(REGISTER_SP, getregval(REGISTER_SP) - 2);
    *((uint16_t*)virt_memory+stackptr) = val;
    return stackptr;
}

uint16_t popw() {
    setregval(REGISTER_SP, getregval(REGISTER_SP) + 2);
    uint32_t stackptr = segcalc(getregval(REGISTER_SS), getregval(REGISTER_SP));
    return *((uint16_t*)virt_memory+stackptr);
}

uint8_t isWriteAllowed(uint32_t addr) {
    if (addr >= 0x100 && addr <= 0x200 && (IP <= 0x100 || IP >= 0x200)) {
        // if out of "bios" region, prevent writes to this area of memory
        return 0;
    }
    return 1;
}

uint16_t readWord(uint8_t seg, uint16_t off) {
    uint32_t real_addr = segcalc(getregval(seg), off);
    return ((uint16_t)virt_memory[real_addr] + ((uint16_t)virt_memory[real_addr+1]<<8));
}

uint8_t readByte(uint8_t seg, uint16_t off) {
    uint32_t real_addr = segcalc(getregval(seg), off);
    return (uint16_t)virt_memory[real_addr];
}

uint16_t swapWord(uint16_t word) {
    return ((word&0xFF00) >> 8) | ((word&0xFF)<<8);
}

void writeWord(uint8_t seg, uint16_t off, uint16_t val) {
    uint32_t real_addr = segcalc(getregval(seg), off);

    if (!isWriteAllowed(real_addr) || !isWriteAllowed(real_addr + 1)) {
        FN_ELOG("ERROR: CPU attempted disallowed write to %04X:%04X\n",getregval(seg),off);
        running = false;
        return;
    }

    virt_memory[real_addr] = (val&0xFF);
    virt_memory[real_addr+1] = (val&0xFF00)>>8;
}

void writeByte(uint8_t seg, uint16_t off, uint8_t val) {
    uint32_t real_addr = segcalc(getregval(seg), off);

    if (!isWriteAllowed(real_addr)) {
        FN_ELOG("ERROR: CPU attempted disallowed write to %04X:%04X\n",getregval(seg),off);
        running = false;
        return;
    }

    virt_memory[real_addr] = val;
}

void setflag(uint16_t FLAGID) {
    FLAGS |= FLAGID;
}

void clearflag(uint16_t FLAGID) {
    FLAGS &= ~FLAGID;
}

uint16_t getflag(uint16_t FLAGID) {
    return (FLAGS & FLAGID);
}

// this is a lazy, likely buggy 8086 emulator.
// i will probably rewrite this in the future
int main() {
    printf("Starting charlie86\n");
    printf("Registers: [");
    for (int i=0;i<0xF;i++) {
        regs[i] = 0;
        printf("%s=%X ", getregname(i), regs[i]);
    }
    printf("IP=%X]\n",IP);

    virt_memory = (unsigned char*) malloc(VIRTUAL_MEMORY_SIZE);
    memset(virt_memory, 0, VIRTUAL_MEMORY_SIZE);

    printf("Starting GUI\n");
    initvideo(80,25,0xB8000, TEXT_MODE_8BPC);
    memset(virt_memory + 0xB8000, ' ', 80*25*2);

    printf("Allocated %d bytes of virtual memory\n", VIRTUAL_MEMORY_SIZE);

    printf("Loading bootsector..\n");
    loadsector(0, 0x7C00,1, BOOT_DISK);
    //loadsector(0, 0x7C00, 4, "../disk/BOOTSEC.BIN");
    loadsector(0, 0x100, 1, "../disk/BOOTROM.BIN"); // load the ROM/(BIOS i guess) at 0x100

    jump(0x100);

    // This is now handled in-VM (in the ROM)
    //if (virt_memory[0x7DFE] != 0x55 || virt_memory[0x7DFF] != 0xAA) {
    //    fprintf(stderr, "Warning: Magic number 55AA not present in bootsector! [%x%x]\n", virt_memory[0x7DFE], virt_memory[0x7DFF]);
    //}

    while (running) {
        uint8_t ipc = readByte(SEG, IP);
        IP++;

        switch (ipc) {
            case OPCODE_MOV_A1_MEMBYTE2AL: { // MOV BYTE AL, [VAL] VAL=IMMEDIATE
                uint8_t d = readByte(SEG, readWord(SEG,IP));
                IP += 2;
                setregval(REGISTER_AL, d);

                regdump();
                FN_ILOG("MOV BYTE AL, [0x%02X]\n", readWord(SEG,IP)-2);
                break;
            }
            case OPCODE_MOV_A1_MEMWORD2AX: { // MOV WORD AX, [VAL] VAL=IMMEDIATE
                uint16_t d = readWord(SEG, readWord(SEG,IP));
                IP += 2;
                setregval(REGISTER_AX, d);

                regdump();
                FN_ILOG("MOV BYTE AX, [0x%04X]\n", readWord(SEG,IP-2));
                break;
            }
            case OPCODE_CMP_3C_BYTEINAL: { // CMP BYTE AL, VAL (IMMEDIATE)
                uint8_t d = readByte(SEG,IP);
                IP++;

                if (d == (uint8_t)getregval(REGISTER_AL))
                    setflag(FLAG_ZERO);
                else
                    clearflag(FLAG_ZERO);

                regdump();
                FN_ILOG("CMP BYTE AL, 0x%02X\n",d);

                break;
            }
            case OPCODE_CMP_3D_WORDINAX: { // CMP WORD AX, VAL (IMMEDIATE)
                uint16_t d = swapWord(readWord(SEG,IP));
                IP+=2;

                if (d == getregval(REGISTER_AX))
                    setflag(FLAG_ZERO);
                else
                    clearflag(FLAG_ZERO);

                regdump();
                FN_ILOG("CMP WORD AX, 0x%04X %X\n",d,getregval(REGISTER_AX));

                break;
            }
            case OPCODE_JA_77_8REL: //TODO: you know what do to future me
            case OPCODE_JE_74_8REL: { // JZ 8 BIT RELATIVE
                sint8_t t = readByte(SEG, IP);
                IP++;

                if (getflag(FLAG_ZERO))
                    jump(IP + t);

                regdump();
                FN_ILOG("JE 0x%04X %s [8rel]\n",IP,getflag(FLAG_ZERO)?"[jumped]":"[not jumped]");
                break;
            }

            case OPCODE_JNE_75_8REL: { // JNZ 8 BIT RELATIVE
                sint8_t t = readByte(SEG, IP);
                IP++;

                if (!getflag(FLAG_ZERO))
                    jump(IP + t);

                regdump();
                FN_ILOG("JNE 0x%04X %s [8rel]\n",IP,getflag(FLAG_ZERO)?"[not jumped]":"[jumped]");
                break;
            }
            case OPCODE_MOV_B0_IMBYTE2AL : // MOV BYTE REG, VAL (NON-STACK, IMMEDIATE)
            case OPCODE_MOV_B4_IMBYTE2AH :
            case OPCODE_MOV_B2_IMBYTE2DL :
            case OPCODE_MOV_B6_IMBYTE2DH :
            case OPCODE_MOV_B1_IMBYTE2CL :
            case OPCODE_MOV_B5_IMBYTE2CH :
            case OPCODE_MOV_B3_IMBYTE2BL :
            case OPCODE_MOV_B7_IMBYTE2BH : {
                uint8_t regid = ipc&0xF; // get register ID from opcode

                setregval(regid, readByte(SEG, IP));
                IP++;

                regdump();
                FN_ILOG("MOV BYTE %s, 0x%02X [IM, b0-b8]\n", getregname(regid), getregval(regid));
                break;
            }
            case OPCODE_MOV_B8_IMWORD2AX : // MOV WORD REG, VAL (NON-STACK, IMMEDIATE)
            case OPCODE_MOV_BB_IMWORD2BX :
            case OPCODE_MOV_B9_IMWORD2CX :
            case OPCODE_MOV_BA_IMWORD2DX :
            case OPCODE_MOV_BC_IMWORD2SP :
            case OPCODE_MOV_BD_IMWORD2BP :
            case OPCODE_MOV_BE_IMWORD2SI :
            case OPCODE_MOV_BF_IMWORD2DI : {
                uint8_t regid = ipc&0xF; // get register ID from opcode

                setregval(regid, readWord(SEG, IP));
                IP+=2;

                regdump();
                FN_ILOG("MOV WORD %s, 0x%04X [IM, b8-bf]\n", getregname(regid), getregval(regid));
                break;
            }
            case OPCODE_MOV_8C_SR2RGWORD:
            case OPCODE_MOV_8E_RGWORD2SR: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                // if d is 1, move rmw to sr, else sr to rmw

                mr.w = 1;

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = sr2regid(mr.rm);
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = sr2regid(mr.reg);
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1:
                        loc = rm2loc(mr.rm);
                        writeWord(SEG_OVERRIDE, loc, getregval(regid1));

                        regdump();
                        FN_ILOG("MOV %s, %s [movsr m0]\n", getrmname(mr.rm), getregname(regid1));
                        break;
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented MOV memory access mode %d\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("MOV %s, %s [movsr m3]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_MOV_C7_MRMOVWORD1: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                // if d is 1, move rmw to sr, else sr to rmw

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = (mr.rm + (mr.w << 3));
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = (mr.reg + (mr.w << 3));
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1: {
                        loc = readWord(SEG, IP);
                        IP+=2;

                        uint16_t val = readWord(SEG, IP);
                        IP+=2;

                        writeWord(SEG_OVERRIDE, loc, val);

                        regdump();
                        FN_ILOG("MOV [0x%04X:0x%04X], 0x%04X [movmr16_RM1]\n",getregval(SEG_OVERRIDE), loc, val);
                        break;
                    }
                    case MR_RM2_DISP8: {
                        sint8_t disp = readByte(SEG, IP);
                        loc = getrmdispval(mr.rm, disp);
                        IP++;

                        uint16_t val = readWord(SEG, IP);
                        IP += 2;

                        writeWord(SEG_OVERRIDE, loc, val);

                        regdump();
                        FN_ILOG("MOV [%s+%d](%04X:%04X), 0x%X [movmr16_RM2_DISP8]\n", getrmnamenb(mr.rm), disp, getregval(SEG_OVERRIDE), loc, val);
                        break;
                    }
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented MOV memory access mode %d [ipcC7]\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("MOV %s, %s [movmr16_2REG]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_MOV_C6_MRMOVBYTE1: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                // if d is 1, move rmw to sr, else sr to rmw

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = (mr.rm + (mr.w << 3));
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = (mr.reg + (mr.w << 3));
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1: {
                        if (mr.rm == 0b110) { // Drc't addition!
                            loc = readWord(SEG, IP);
                            IP+=2;

                            uint8_t val = readByte(SEG, IP);
                            IP++;

                            writeByte(SEG_OVERRIDE, loc, val);

                            regdump();
                            FN_ILOG("MOV [0x%04X:0x%04X], 0x%02X [movmr8_RM1]\n",
                                    getregval(SEG_OVERRIDE), loc, val);

                            break;
                        } else {
                            loc = rm2loc(mr.rm);

                            uint8_t val = readByte(SEG, IP);
                            IP++;

                            writeByte(SEG_OVERRIDE, loc, val);

                            regdump();
                            FN_ILOG("MOV %s[0x%04X:0x%04X], 0x%02X [movmr8_RM1]\n", getrmname(mr.rm),
                                    getregval(SEG_OVERRIDE), loc, val);
                            break;
                        }
                    }
                    case MR_RM2_DISP8: {
                        sint8_t disp = readByte(SEG, IP);
                        loc = getrmdispval(mr.rm, disp);
                        IP++;

                        uint8_t val = readByte(REGISTER_CS, IP);
                        IP ++;

                        writeByte(SEG_OVERRIDE, loc, val);

                        regdump();
                        FN_ILOG("MOV [%s+%d](%04X:%04X), 0x%X [movmr8_RM2_DISP8]\n", getrmnamenb(mr.rm), (sint8_t)disp, getregval(SEG_OVERRIDE), loc, val);
                        break;
                    }
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented MOV memory access mode %d [ipcC6]\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("MOV %s, %s [movmr8_2REG]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_MOV_88_MRMOVBYTE2: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                // if d is 1, move rmw to sr, else sr to rmw
                mr.w = 0;

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = (mr.rm + (mr.w << 3));
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = (mr.reg + (mr.w << 3));
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1: {
                        loc = rm2loc(mr.rm);
                        uint8_t reg = (mr.reg + (mr.w << 3));

                        if (mr.w) {
                            writeWord(SEG_OVERRIDE, loc, getregval(reg));
                        } else {
                            writeByte(SEG_OVERRIDE, loc, getregval(reg));
                        }

                        regdump();
                        FN_ILOG("MOV %s(%04X:%04X), %s(%04X) [w=%d]\n", getrmname(mr.rm), getregval(SEG_OVERRIDE), loc, getregname(reg), getregval(reg), mr.w);

                        break;
                    }
                    case MR_RM2_DISP8: {
                        loc = rm2loc(mr.rm);
                        loc += readByte(SEG, IP);
                        IP++;
                        uint8_t reg = (mr.reg + (mr.w << 3));

                        if (mr.w) {
                            writeWord(SEG_OVERRIDE, loc, getregval(reg));
                        } else {
                            writeByte(SEG_OVERRIDE, loc, getregval(reg));
                        }

                        regdump();
                        FN_ILOG("MOV %s(%04X:%04X), %s(%04X) [w=%d, disp8]\n", getrmname(mr.rm), getregval(SEG_OVERRIDE), loc, getregname(reg), getregval(reg), mr.w);

                        break;
                    }
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented MOV memory access mode %d [ipc88]\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("MOV BYTE %s, %s [movmr8_2REG]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_MOV_8A_MRMOVBYTE1: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                // if d is 1, move rmw to sr, else sr to rmw
                mr.w = 0;

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = (mr.rm + (mr.w << 3));
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = (mr.reg + (mr.w << 3));
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1: {
                        loc = rm2loc(mr.rm);
                        uint8_t reg = (mr.reg + (mr.w << 3));

                        if (mr.w) {
                            //writeWord(SEG_OVERRIDE, loc, getregval(reg));
                            setregval(reg, readWord(SEG_OVERRIDE, loc));
                        } else {
                            //writeByte(SEG_OVERRIDE, loc, getregval(reg));
                            setregval(reg, readByte(SEG_OVERRIDE, loc));
                        }

                        regdump();
                        FN_ILOG("MOV %s(%04X), %s(%04X:%04X)  [w=%d]\n",  getregname(reg), getregval(reg), getrmname(mr.rm), getregval(SEG_OVERRIDE), loc, mr.w);

                        break;
                    }
                    case MR_RM2_DISP8: {
                        loc = rm2loc(mr.rm);
                        loc += readByte(SEG, IP);
                        IP++;
                        uint8_t reg = (mr.reg + (mr.w << 3));

                        if (mr.w) {
                            //writeWord(SEG_OVERRIDE, loc, getregval(reg));
                            setregval(reg, readWord(SEG_OVERRIDE, loc));
                        } else {
                            //writeByte(SEG_OVERRIDE, loc, getregval(reg));
                            setregval(reg, readByte(SEG_OVERRIDE, loc));
                        }

                        regdump();
                        FN_ILOG("MOV  %s(%04X), %s(%04X:%04X) [w=%d, disp8]\n", getregname(reg), getregval(reg), getrmname(mr.rm), getregval(SEG_OVERRIDE), loc,  mr.w);

                        break;
                    }
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented MOV memory access mode %d [ipc88]\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("MOV BYTE %s, %s [movmr8_2REG]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_INT: {
                interrupt(readByte(SEG, IP));
                IP++;

                regdump();
                FN_ILOG("INT %02X\n", readByte(SEG, IP-1));
                break;
            }
            case OPCODE_BREAKPOINT: {
                interrupt(3);

                regdump();
                FN_ILOG("INT 3 [breakpoint]\n");
                break;
            }
            case OPCODE_NOP: {
                regdump();
                FN_ILOG("NOP\n");
                break;
            }
            case OPCODE_JMP_REL16: {
                sint16_t targ = readWord(SEG, IP);
                IP+=2;

                jump(IP + targ);

                regdump();
                FN_ILOG("JMP SHORT 0x%04X [rel16]\n",IP);
                break;
            }
            case OPCODE_JMP_REL8: {
                sint8_t targ = readByte(SEG, IP);
                IP++;

                jump(IP + targ);

                regdump();
                FN_ILOG("JMP SHORT 0x%02X [rel8]\n",IP);
                break;
            }
            case OPCODE_LODSB: {
                uint8_t v = readByte(SEG_OVERRIDE, getregval(REGISTER_SI));
                setregval(REGISTER_AL, v);

                if (getflag(FLAG_DIRECTION))
                    setregval(REGISTER_SI, getregval(REGISTER_SI)-1);
                else
                    setregval(REGISTER_SI, getregval(REGISTER_SI)+1);

                regdump();
                FN_ILOG("LODSB (%02X from %04X:%04X)\n", getregval(REGISTER_AL), getregval(SEG_OVERRIDE), getregval(REGISTER_SI));
                break;
            }
            case OPCODE_LODSW: {
                uint16_t v = readWord(SEG_OVERRIDE, getregval(REGISTER_SI));
                setregval(REGISTER_AX, v);

                if (getflag(FLAG_DIRECTION))
                    setregval(REGISTER_SI, getregval(REGISTER_SI)-2);
                else
                    setregval(REGISTER_SI, getregval(REGISTER_SI)+2);

                regdump();
                FN_ILOG("LODSW (%04X from %04X:%04X)\n", getregval(REGISTER_AX), getregval(SEG_OVERRIDE), getregval(REGISTER_SI));
                break;
            }
            case OPCODE_OR_09_WORD: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                // if d is 1, move rmw to sr, else sr to rmw

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = (mr.rm + (mr.w << 3));
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = (mr.reg + (mr.w << 3));
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1:
                        loc = rm2loc(mr.rm);
                        writeWord(SEG_OVERRIDE, loc, getregval(regid1) | readWord(SEG_OVERRIDE, loc));

                        regdump();

                        FN_ILOG("OR %s, %s [ipc09, m0]\n", getrmname(mr.rm), getregname(regid1));
                        break;
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented OR memory access mode %d [ipc09]\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1) | getregval(regid2));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("OR %s, %s [ipc09, m3]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_OR_08_BYTE: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                mr.w = 0;

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = (mr.rm+ (mr.w << 3));
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = (mr.reg+ (mr.w << 3));
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1:
                        loc = rm2loc(mr.rm);
                        writeByte(SEG_OVERRIDE, loc, getregval(regid1) | readByte(SEG_OVERRIDE, loc));
                        if (readByte(SEG_OVERRIDE, loc) == 0)
                            setflag(FLAG_ZERO);
                        else
                            clearflag(FLAG_ZERO);

                        regdump();
                        FN_ILOG("OR %s, %s [ipc08]\n", getrmname(mr.rm), getregname(regid1));
                        break;
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                        FN_ELOG("FIXME: Unimplemented OR memory access mode %d\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1) | getregval(regid2));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        else
                            clearflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("OR %s, %s [ipc08]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_XOR_31_WORD: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                mr.w = 1;

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = (mr.rm + (mr.w << 3));
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = (mr.reg+ (mr.w << 3));
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1:
                        loc = rm2loc(mr.rm);
                        writeByte(SEG_OVERRIDE, loc, getregval(regid1) ^ readByte(SEG_OVERRIDE, loc));
                        if (readByte(SEG_OVERRIDE, loc) == 0)
                            setflag(FLAG_ZERO);
                        else
                            clearflag(FLAG_ZERO);

                        regdump();
                        FN_ILOG("XOR %s, %s [ipc31 m0]\n", getrmname(mr.rm), getregname(regid1));
                        break;
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                        FN_ELOG("FIXME: Unimplemented XOR memory access mode %d\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1) ^ getregval(regid2));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        else
                            clearflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("XOR %s, %s [ipc31 m3]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_STI: {
                setflag(FLAG_INT);

                regdump();
                FN_ILOG("STI\n");
                break;
            }
            case OPCODE_CLI: {
                clearflag(FLAG_INT);

                regdump();
                FN_ILOG("CLI\n");
                break;
            }
            case OPCODE_HLT: {
                running = false;

                regdump();
                FN_ILOG("HLT\n");
                break;
            }
            case OPCODE_CLD: {
                clearflag(FLAG_DIRECTION);

                regdump();
                FN_ILOG("CLD\n");
                break;
            }
            case OPCODE_ADD_83_8IM: { // OPCODE_CMP_83_8IM also
                if ((readByte(SEG, IP)&0b111000) == 0) { // ADD reg, si1
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    //mr.w = 0;

                    uint8_t regid2 = mr.rm + (mr.w<<3);

                    switch (mr.mod) {
                        case 0:
                        case 1:
                        case 2:
                            FN_ELOG("FIXME: Unimplemented ADD IL mode %d\n",mr.mod);
                            running = false;
                            break;
                        case 3:
                            setregval(regid2, getregval(regid2) + (sint8_t)readByte(SEG, IP));
                            IP+=1;

                            regdump();
                            FN_ILOG("ADD %s, %d [ipc83 m3]\n", getregname(regid2), (sint8_t )readByte(SEG, IP-1));
                    }

                    break;
                }

                if ((readByte(SEG, IP)&0b111000) == 0b111000) { // CMP reg, si1
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    switch (mr.mod) {
                        case 0:
                        case 1:
                        case 2:
                            FN_ELOG("FIXME: Unimplemented CMP IL mode %d\n",mr.mod);
                            running = false;
                            break;
                        case 3: {
                            uint16_t regid = mr.rm + (mr.w << 3);

                            if (((sint16_t )getregval(regid) - (sint8_t )readByte(SEG, IP)) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);
                            IP += 1;

                            regdump();
                            FN_ILOG("CMP %s, %d[%02X] [ipc83 m3 8bit dv=%d]\n", getregname(regid), (sint8_t) readByte(SEG, IP - 1), readByte(SEG, IP - 1), (sint16_t) getregval(regid));
                        }
                    }

                    break;
                }

                if (((readByte(SEG, IP)&0b111000)>>3) == 5) { // SUB reg, si1
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    //mr.w = 0;

                    uint8_t regid2 = mr.rm + (mr.w<<3);

                    switch (mr.mod) {
                        case 0:
                        case 1:
                        case 2:
                            FN_ELOG("FIXME: Unimplemented SUB IL mode %d\n",mr.mod);
                            running = false;
                            break;
                        case 3:
                            setregval(regid2, getregval(regid2) - (sint8_t)readByte(SEG, IP));
                            IP+=1;

                            regdump();
                            FN_ILOG("SUB %s, %d [ipc83 m3]\n", getregname(regid2), (sint8_t )readByte(SEG, IP-1));
                    }

                    break;
                }
                break;
            }
            case OPCODE_ADD_81_16IM: { // OPCODE_CMP_81_16IM also
                if ((readByte(SEG, IP)&0b111000) == 0) { // ADD reg, si1
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    mr.w = 1;

                    uint8_t regid2 = mr.rm + (mr.w<<3);

                    switch (mr.mod) {
                        case 0:
                        case 1:
                        case 2:
                            FN_ELOG("FIXME: Unimplemented ADD IL mode %d\n",mr.mod);
                            running = false;
                            break;
                        case 3:
                            setregval(regid2, getregval(regid2) + (sint16_t)readWord(SEG, IP));
                            IP+=2;

                            regdump();
                            FN_ILOG("ADD %s, %d [ipc83 16bit m3]\n", getregname(regid2), (sint16_t )readWord(SEG, IP-2));
                    }

                    break;
                }

                if ((readByte(SEG, IP)&0b111000) == 0b111000) { // CMP reg, si1
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    mr.w = 1;

                    switch (mr.mod) {
                        case 0:
                        case 1:
                        case 2:
                            FN_ELOG("FIXME: Unimplemented CMP IL mode %d\n",mr.mod);
                            running = false;
                            break;
                        case 3: {
                            uint16_t regid = mr.rm + (mr.w << 3);

                            if (((sint16_t )getregval(regid) - (sint16_t )readWord(SEG, IP)) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);
                            IP += 2;

                            regdump();
                            FN_ILOG("CMP %s, %d[%04X] [ipc83 m3 16bit dv=%d]\n", getregname(regid), (sint16_t) readWord(SEG, IP - 2), readWord(SEG, IP - 2), (sint16_t) getregval(regid));
                        }
                    }

                    break;
                }

                break;
            }
            case OPCODE_CALL_E8_NEAR: {
                sint16_t targ = (sint16_t)readWord(SEG, IP);
                IP+=2;

                uint16_t nip = (uint16_t)((sint16_t)IP + targ);

                call_xhandler(getregval(SEG), nip, IP);

                pushw(IP);
                jump(nip);

                regdump();
                FN_ILOG("CALL NEAR 0x%04X:0x%04X %d\n", getregval(SEG), nip, targ);
                break;
            }
            case OPCODE_MUL_F6_BYTE: {
                if ((readByte(SEG, IP)&0b111000) == 4) {
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    printf("ERROR: Unimplemented instruction OPCODE_MUL_F6_BYTE!\n");
                    running = false;

                    break;
                }
            }
            case OPCODE_MUL_F7_WORD: {
                if (((readByte(SEG, IP)&0b111000)>>3) == 4) {
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    mr.w = 1;

                    switch (mr.mod) {
                        case MR_RM1:
                        case MR_RM2_DISP8:
                        case MR_RM2_DISP16:
                            FN_ELOG("FIXME: Unimplemented MUL memory access mode %d\n", mr.mod);
                            running = false;
                            break;
                        case MR_2REG: {
                            uint8_t regid = mr.rm + 8;
                            setregval(REGISTER_AX, getregval(REGISTER_AX) * getregval(regid));

                            regdump();
                            FN_ILOG("MUL %s[%d]\n", getregname(regid), getregval(regid));
                            break;
                        }
                    }

                    break;
                }
            }
            case OPCODE_MOV_89_GR2RGWORD:
            case OPCODE_MOV_8B_RGWORD2GR: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                // if d is 1, move rmw to sr, else sr to rmw

                mr.w = 1;

                uint8_t regid1,regid2;
                if (!mr.d) {
                    regid1 = (mr.reg + (mr.w << 3));
                    regid2 = (mr.rm + (mr.w << 3));
                } else {
                    regid1 = (mr.rm + (mr.w << 3));
                    regid2 = (mr.reg + (mr.w << 3));
                }

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1:
                        if (mr.rm == 0b110) { // Drc't addition! (ree)
                            loc = readWord(SEG, IP);
                            IP+=2;

                            writeWord(SEG_OVERRIDE, loc, getregval(regid1));

                            regdump();
                            FN_ILOG("MOV [0x%04X], %s [movgr m0 drc't]\n", loc, getregname(regid1));
                        } else {
                            loc = rm2loc(mr.rm);
                            writeWord(SEG_OVERRIDE, loc, getregval(regid1));

                            regdump();
                            FN_ILOG("MOV %s, %s [movgr m0]\n", getrmname(mr.rm), getregname(regid1));
                        }
                        break;
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented MOV memory access mode %d [ipc89/8B]\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        setregval(regid2, getregval(regid1));
                        if (getregval(regid2) == 0)
                            setflag(FLAG_ZERO);
                        regdump();
                        FN_ILOG("MOV %s, %s [movgr m3]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_PUSH_AX_REG:
            case OPCODE_PUSH_BX_REG:
            case OPCODE_PUSH_CX_REG:
            case OPCODE_PUSH_DX_REG:
            case OPCODE_PUSH_BP_REG:
            case OPCODE_PUSH_SP_REG:
            case OPCODE_PUSH_SI_REG:
            case OPCODE_PUSH_DI_REG: {
                uint8_t regid = (ipc - 0x50) + 8;

                pushw(getregval(regid));

                regdump();
                FN_ILOG("PUSH %s(%04X) [regpush 50-57]\n",getregname(regid),getregval(regid));
                break;
            }
            case OPCODE_POP_AX_REG:
            case OPCODE_POP_BX_REG:
            case OPCODE_POP_CX_REG:
            case OPCODE_POP_DX_REG:
            case OPCODE_POP_BP_REG:
            case OPCODE_POP_SP_REG:
            case OPCODE_POP_SI_REG:
            case OPCODE_POP_DI_REG: {
                uint8_t regid = (ipc - 0x58) + 8;

                setregval(regid, popw());

                regdump();
                FN_ILOG("POP %s(%04X) [regpop 58-5F]\n",getregname(regid), getregval(regid));
                break;
            }
            case OPCODE_CMP_38_8D:
            case OPCODE_CMP_39_16D: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                mr.w = (ipc==OPCODE_CMP_39_16D)?1:0;

                uint8_t regid1 = mr.reg | (mr.w << 3);
                uint8_t regid2 = mr.rm  | (mr.w << 3);

                switch (mr.mod) {
                    case MR_RM1:
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                        FN_ELOG("FIXME: Unimplemented CMP memory access mode %d\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        if (mr.w) {
                            if (((sint16_t) getregval(regid2) - (sint16_t) getregval(regid1)) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);
                            regdump();
                            FN_ILOG("CMP WORD %s, %s [cmp16_2REG ipc38/39 v=%d]\n", getregname(regid2),
                                   getregname(regid1), ((sint16_t) getregval(regid2) - (sint16_t) getregval(regid1)));
                        } else {
                            if (((sint8_t) getregval(regid2) - (sint8_t) getregval(regid1)) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);
                            regdump();
                            FN_ILOG("CMP BYTE %s, %s [cmp8_2REG ipc38/39 v=%d]\n", getregname(regid2),
                                   getregname(regid1), ((sint8_t) getregval(regid2) - (sint8_t) getregval(regid1)));

                        }
                }

                break;
            }
            case OPCODE_ADD_04_ALIM:
            case OPCODE_ADD_05_AXIM: {
                uint8_t wide = ipc&1;

                if (wide) {
                    setregval(REGISTER_AX, getregval(REGISTER_AX) + readWord(SEG, IP));
                    IP+=2;

                    if (getregval(REGISTER_AX)==0)
                        setflag(FLAG_ZERO);
                    else
                        clearflag(FLAG_ZERO);

                    if (getregval(REGISTER_AX) < readWord(SEG, IP-2))
                        setflag(FLAG_OVERFLOW);

                    regdump();
                    FN_ILOG("ADD AX, 0x%04X [ipc05]\n", readWord(SEG,IP-2));
                } else {
                    setregval(REGISTER_AL, getregval(REGISTER_AL) + readByte(SEG,IP));
                    IP++;

                    if (getregval(REGISTER_AL)==0)
                        setflag(FLAG_ZERO);
                    else
                        clearflag(FLAG_ZERO);

                    if (getregval(REGISTER_AL) < readByte(SEG, IP-1))
                        setflag(FLAG_OVERFLOW);

                    regdump();
                    FN_ILOG("ADD AL, 0x%02X [ipc04]\n", readByte(SEG,IP-1));
                }

                break;
            }
            case OPCODE_RET: {
                jump(popw());

                regdump();
                FN_ILOG("RET 0x%04X:0x%04X\n",getregval(SEG), IP);

                if ((getregval(SEG) == 0x0000 && IP == 0x0000)) { // something is without a doubt wrong if this is triggered.
                    uint16_t segp = getregval(SEG);
                    uint16_t ipp = IP;

                    callstack_entry_t e = callstack.top();
                    setregval(SEG, e.second.first);
                    jump(e.second.second);

                    FN_ELOG("WARNING: Prevented RET to %04X:%04X, taking return value off callstack. [%04X:%04X]\n", segp, ipp, getregval(SEG), IP);
                }

                ret_xhandler(getregval(SEG), IP);

                break;
            }
            case OPCODE_INC_FE_8MR: {
                if ((readWord(SEG, IP)&0b111000) == 0b000000) { // inc
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    mr.w = 0;
                    uint8_t regid = mr.rm | (mr.w << 3);

                    switch (mr.mod) {
                        case MR_RM1: {
                            uint8_t targ = readByte(SEG, IP);
                            IP++;

                            writeByte(SEG_OVERRIDE, targ, readWord(SEG_OVERRIDE, targ) + 1);

                            regdump();
                            FN_ILOG("INC BYTE [0x%04X]\n", targ);

                            break;
                        }
                        case MR_RM2_DISP8:
                        case MR_RM2_DISP16:
                            FN_ELOG("FIXME: Unimplemented INC memory access mode %d [ipcFE/FF]\n", mr.mod);
                            running = false;
                            break;
                        case MR_2REG: {
                            if ((getregval(regid) + 1) < getregval(regid))
                                setflag(FLAG_OVERFLOW);

                            setregval(regid, getregval(regid) + 1);
                            regdump();
                            FN_ILOG("INC %s [ipcFE/FF]\n", getregname(regid));
                        }
                    }
                }
                if ((readWord(SEG, IP)&0b111000) == 0b001000) { // dec
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    mr.w = 0;
                    uint8_t regid = mr.rm | (mr.w << 3);

                    switch (mr.mod) {
                        case MR_RM1: {
                            uint8_t targ = readByte(SEG, IP);
                            IP++;

                            writeByte(SEG_OVERRIDE, targ, readWord(SEG_OVERRIDE, targ) - 1);

                            if (readWord(SEG_OVERRIDE, targ)  == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            regdump();
                            FN_ILOG("DEC BYTE [0x%04X]\n", targ);

                            break;
                        }
                        case MR_RM2_DISP8:
                        case MR_RM2_DISP16:
                        FN_ELOG("FIXME: Unimplemented DEC memory access mode %d [ipcFE]\n", mr.mod);
                            running = false;
                            break;
                        case MR_2REG: {
                            if ((getregval(regid) - 1) > getregval(regid))
                                setflag(FLAG_OVERFLOW);

                            setregval(regid, getregval(regid) - 1);

                            if (getregval(regid)  == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            regdump();
                            FN_ILOG("DEC %s [ipcFE]\n", getregname(regid));
                        }
                    }
                }
                if ((readWord(SEG, IP)&0b111000) == 0b001000) {
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    mr.w = 0;
                    uint8_t regid = mr.rm | (mr.w << 3);

                    switch (mr.mod) {
                        case MR_RM1: {
                            uint8_t targ = readByte(SEG, IP);
                            IP++;

                            writeByte(SEG_OVERRIDE, targ, readWord(SEG_OVERRIDE, targ) - 1);

                            if (readWord(SEG_OVERRIDE, targ)  == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            regdump();
                            FN_ILOG("DEC BYTE [0x%04X]\n", targ);

                            break;
                        }
                        case MR_RM2_DISP8:
                        case MR_RM2_DISP16:
                            FN_ELOG("FIXME: Unimplemented INC memory access mode %d [ipcFE/FF]\n", mr.mod);
                            running = false;
                            break;
                        case MR_2REG: {
                            if ((getregval(regid) - 1) > getregval(regid))
                                setflag(FLAG_OVERFLOW);

                            setregval(regid, getregval(regid) - 1);

                            if (getregval(regid)  == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            regdump();
                            FN_ILOG("DEC %s [ipcFE/FF]\n", getregname(regid));
                        }
                    }
                }

                break;
            }
            case OPCODE_INC_FF_16MR: {
                if ((readWord(SEG, IP)&0b111000) == 0b000000) {
                    mrfield mr = parsemr(IP, ipc);
                    IP++;

                    mr.w = 1;
                    uint8_t regid = mr.rm | (mr.w << 3);

                    switch (mr.mod) {
                        case MR_RM1: {
                            uint8_t targ = readByte(SEG, IP);
                            IP++;

                            writeWord(SEG_OVERRIDE, targ, readWord(SEG_OVERRIDE, targ) + 1);

                            regdump();
                            FN_ILOG("INC WORD [0x%04X]\n", targ);

                            break;
                        }
                        case MR_RM2_DISP8:
                        case MR_RM2_DISP16:
                        FN_ELOG("FIXME: Unimplemented INC memory access mode %d [ipcFE/FF]\n", mr.mod);
                            running = false;
                            break;
                        case MR_2REG: {
                            if ((getregval(regid) + 1) < getregval(regid))
                                setflag(FLAG_OVERFLOW);

                            setregval(regid, getregval(regid) + 1);
                            regdump();
                            FN_ILOG("INC %s [ipcFF]\n", getregname(regid));
                        }
                    }
                }
                break;
            }
            case OPCODE_CMP_80_16IM: {
                mrfield mr = parsemr(IP, ipc);
                if (mr.reg == 7) {
                    IP++;

                    uint8_t regid = mr.rm | (mr.w << 3);

                    switch (mr.mod) {
                        case MR_RM1: {
                            uint16_t loc = rm2loc(mr.rm);
                            uint8_t term = readByte(SEG, IP);
                            IP++;

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) - (sint8_t)term) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) - (sint8_t)term) < 0)
                                setflag(FLAG_SIGN);
                            else
                                clearflag(FLAG_SIGN);

                            lastcmp = 0;
                            lastcmp += ((sint8_t)readByte(SEG_OVERRIDE, loc) - (sint8_t)term);

                            regdump();
                            FN_ILOG("CMP BYTE %s(%02X), %02X [ipc%02X e=%d s=%d]\n", getrmname(mr.rm), readByte(SEG_OVERRIDE, loc), term, ipc, getflag(FLAG_ZERO), getflag(FLAG_SIGN));

                            break;
                        }
                        case MR_RM2_DISP8:
                        case MR_RM2_DISP16:
                            FN_ELOG("FIXME: Unimplemented CMP memory access mode %d [ipc80]\n", mr.mod);
                            running = false;
                            break;
                        case MR_2REG: {
                            if (!mr.w) {
                                uint8_t term = readByte(SEG, IP);
                                IP++;

                                if (((sint8_t)getregval(regid) - (sint8_t)term) == 0)
                                    setflag(FLAG_ZERO);
                                else
                                    clearflag(FLAG_ZERO);

                                regdump();
                                FN_ILOG("CMP %s, %02X [ipc80, v=%d]\n", getregname(regid), term, ((sint8_t)getregval(regid) - (sint8_t)term));
                            }
                        }
                    }

                    break;
                } else if (mr.reg == 0) {
                    IP++;

                    uint8_t regid = mr.rm | (mr.w << 3);

                    switch (mr.mod) {
                        case MR_RM1: {
                            uint16_t loc = rm2loc(mr.rm);
                            uint8_t term = readByte(SEG, IP);
                            IP++;

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) + (sint8_t)term) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) + (sint8_t)term) < 0)
                                setflag(FLAG_SIGN);
                            else
                                clearflag(FLAG_SIGN);

                            writeByte(SEG_OVERRIDE, loc, ((sint8_t)readByte(SEG_OVERRIDE, loc) + (sint8_t)term));

                            regdump();
                            FN_ILOG("ADD BYTE %s(%02X), %02X [ipc%02X e=%d s=%d]\n", getrmname(mr.rm), readByte(SEG_OVERRIDE, loc), term, ipc, getflag(FLAG_ZERO), getflag(FLAG_SIGN));

                            break;
                        }
                        case MR_RM2_DISP8:
                        case MR_RM2_DISP16:
                        FN_ELOG("FIXME: Unimplemented CMP memory access mode %d [ipc80]\n", mr.mod);
                            running = false;
                            break;
                        case MR_2REG: {
                            if (!mr.w) {
                                uint8_t term = readByte(SEG_OVERRIDE, IP);
                                IP++;

                                setregval(regid, getregval(regid) + term);

                                regdump();
                                FN_ILOG("ADD %s, %02X [ipc80, v=%d]\n", getregname(regid), term, getregval(regid));
                            }
                        }
                    }
                    break;
                } else if (mr.reg == 5) {
                    IP++;

                    mr.w = 0;
                    uint8_t regid = mr.rm | (mr.w << 3);

                    switch (mr.mod) {
                        case MR_RM1: {
                            uint16_t loc = rm2loc(mr.rm);
                            uint8_t term = readByte(SEG, IP);
                            IP++;

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) - (sint8_t)term) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) - (sint8_t)term) < 0)
                                setflag(FLAG_SIGN);
                            else
                                clearflag(FLAG_SIGN);

                            writeByte(SEG_OVERRIDE, loc, ((sint8_t)readByte(SEG_OVERRIDE, loc) - (sint8_t)term));

                            regdump();
                            FN_ILOG("SUB BYTE %s(%02X), %02X [ipc%02X e=%d s=%d mr_rm1]\n", getrmname(mr.rm), readByte(SEG_OVERRIDE, loc), term, ipc, getflag(FLAG_ZERO), getflag(FLAG_SIGN));

                            break;
                        }
                        case MR_RM2_DISP8:
                        case MR_RM2_DISP16:
                        FN_ELOG("FIXME: Unimplemented SUB memory access mode %d [ipc80]\n", mr.mod);
                            running = false;
                            break;
                        case MR_2REG: {
                            if (!mr.w) {
                                uint8_t term = readByte(SEG_OVERRIDE, IP);
                                IP++;

                                setregval(regid, getregval(regid) - term);

                                regdump();
                                FN_ILOG("SUB %s, %02X [ipc80, v=%d mr_2reg]\n", getregname(regid), term, getregval(regid));
                            }
                        }
                    }

                    //exit(0);
                    //break;
                } else if (mr.reg == 1) { // OR
                    IP++;

                    uint8_t regid = mr.rm | (mr.w << 3);

                    switch (mr.mod) {
                        case MR_RM1: {
                            uint16_t loc = rm2loc(mr.rm);
                            uint8_t term = readByte(SEG, IP);
                            IP++;

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) | (sint8_t)term) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) | (sint8_t)term) < 0)
                                setflag(FLAG_SIGN);
                            else
                                clearflag(FLAG_SIGN);

                            writeByte(SEG_OVERRIDE, loc, (readByte(SEG_OVERRIDE, loc) | term));

                            regdump();;
                            FN_ILOG("OR BYTE %s(%02X), %02X [ipc%02X e=%d s=%d]\n", getrmname(mr.rm), readByte(SEG_OVERRIDE, loc), term, ipc, getflag(FLAG_ZERO), getflag(FLAG_SIGN));

                            break;
                        }
                        case MR_RM2_DISP8:
                            FN_ELOG("FIXME: Unimplemented OR memory access mode %d [ipc80]\n", mr.mod);
                            running = false;
                            break;
                        case MR_RM2_DISP16: {
                            uint16_t loc = rm2loc(mr.rm);
                            uint8_t term = readByte(SEG, IP);
                            IP++;
                            sint16_t off = readWord(SEG, IP);
                            IP+=2;

                            loc += off;

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) | (sint8_t)term) == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            if (((sint8_t)readByte(SEG_OVERRIDE, loc) | (sint8_t)term) < 0)
                                setflag(FLAG_SIGN);
                            else
                                clearflag(FLAG_SIGN);

                            writeByte(SEG_OVERRIDE, loc, (readByte(SEG_OVERRIDE, loc) | term));

                            regdump();;
                            FN_ILOG("OR BYTE [%s+%d](%02X), %02X [ipc%02X e=%d s=%d]\n", getrmnamenb(mr.rm), off, readByte(SEG_OVERRIDE, loc), term, ipc, getflag(FLAG_ZERO), getflag(FLAG_SIGN));

                            break;
                        }
                        case MR_2REG: {
                            if (!mr.w) {
                                uint8_t term = readByte(SEG, IP);
                                IP++;

                                if (((sint8_t)getregval(regid) | (sint8_t)term) == 0)
                                    setflag(FLAG_ZERO);
                                else
                                    clearflag(FLAG_ZERO);

                                setregval(regid,getregval(regid) | term);

                                regdump();
                                FN_ILOG("OR %s, %02X [ipc80, v=%d]\n", getregname(regid), term, ((sint8_t)getregval(regid) - (sint8_t)term));
                            }
                        }
                    }

                    break;
                }

                break;
            }
            case OPCODE_INC_AX_REG:
            case OPCODE_INC_CX_REG:
            case OPCODE_INC_DX_REG:
            case OPCODE_INC_BX_REG:
            case OPCODE_INC_SP_REG:
            case OPCODE_INC_BP_REG:
            case OPCODE_INC_SI_REG:
            case OPCODE_INC_DI_REG: {
                uint8_t reg = (ipc - 0x40) + 8;

                if ((getregval(reg) + 1) < getregval(reg))
                    setflag(FLAG_OVERFLOW);

                setregval(reg, getregval(reg) + 1);

                regdump();
                FN_ILOG("INC %s\n", getregname(reg));

                break;
            }
            case OPCODE_ADD_00_8RM:
            case OPCODE_ADD_01_16RM: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                switch (mr.mod) {
                    case MR_RM1: {
                        uint16_t loc = rm2loc(mr.rm);
                        uint8_t term = readByte(SEG, IP);
                        //IP++;

                        if (((sint8_t)readByte(SEG_OVERRIDE, loc) + (sint8_t)term) == 0)
                            setflag(FLAG_ZERO);
                        else
                            clearflag(FLAG_ZERO);

                        if (((sint8_t)readByte(SEG_OVERRIDE, loc) + (sint8_t)term) < 0)
                            setflag(FLAG_SIGN);
                        else
                            clearflag(FLAG_SIGN);

                        writeByte(SEG_OVERRIDE, loc, ((sint8_t)readByte(SEG_OVERRIDE, loc) + (sint8_t)term));

                        regdump();
                        FN_ILOG("ADD BYTE %s(%02X), %02X [ipc%02X e=%d s=%d]\n", getrmname(mr.rm), readByte(SEG_OVERRIDE, loc), term, ipc, getflag(FLAG_ZERO), getflag(FLAG_SIGN));

                        break;
                    }
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented ADD memory access mode %d [ipc00/01]\n", mr.mod);
                        running = false;
                        break;
                    case MR_2REG: {
                        uint8_t regid1 = (mr.rm + (mr.w<<3));
                        uint8_t regid2 = (mr.reg + (mr.w<<3));

                        if (!mr.w && getregval(regid2) + getregval(regid1)>0xFF)
                            setflag(FLAG_OVERFLOW);

                        setregval(regid1, getregval(regid2) + getregval(regid1));

                        regdump();
                        FN_ILOG("ADD %s, %s [ipc%02X, v=%d, w=%d]\n", getregname(regid1), getregname(regid2), ipc, getregval(regid1), mr.w);
                    }
                }

                break;
            }
            case OPCODE_PUSH_CS_REG: {
                pushw(getregval(REGISTER_CS));

                regdump();
                FN_ILOG("PUSH CS(%04X) [regpush %02X]\n",getregval(REGISTER_CS), ipc);
                break;
            }
            case OPCODE_PUSH_DS_REG: {
                pushw(getregval(SEG_OVERRIDE));

                regdump();
                FN_ILOG("PUSH DS(%04X) [regpush %02X]\n",getregval(SEG_OVERRIDE), ipc);
                break;
            }
            case OPCODE_PUSH_ES_REG: {
                pushw(getregval(REGISTER_ES));

                regdump();
                FN_ILOG("PUSH ES(%04X) [regpush %02X]\n",getregval(REGISTER_ES), ipc);
                break;
            }
            case OPCODE_PUSH_SS_REG: {
                pushw(getregval(REGISTER_SS));

                regdump();
                FN_ILOG("PUSH SS(%04X) [regpush %02X]\n",getregval(REGISTER_SS), ipc);
                break;
            }
            case OPCODE_POP_DS_REG: {
                uint8_t regid = SEG_OVERRIDE;

                setregval(regid, popw());

                regdump();
                FN_ILOG("POP %s(%04X) [regpop %02X]\n",getregname(regid), getregval(regid), ipc);
                break;
            }
            case OPCODE_POP_SS_REG: {
                uint8_t regid = REGISTER_SS;

                setregval(regid, popw());

                regdump();
                FN_ILOG("POP %s(%04X) [regpop %02X]\n",getregname(regid), getregval(regid), ipc);
                break;
            }
            case OPCODE_POP_ES_REG: {
                uint8_t regid = REGISTER_ES;

                setregval(regid, popw());

                regdump();
                FN_ILOG("POP %s(%04X) [regpop %02X]\n",getregname(regid), getregval(regid), ipc);
                break;
            }
            case OPCODE_STOSB: {
                writeByte(REGISTER_ES, getregval(REGISTER_DI), getregval(REGISTER_AL));

                if (getflag(FLAG_DIRECTION))
                    setregval(REGISTER_DI, getregval(REGISTER_DI) - 1);
                else
                    setregval(REGISTER_DI, getregval(REGISTER_DI) + 1);

                regdump();
                FN_ILOG("STOSB (%02X into %04X:%04X)\n", getregval(REGISTER_AL), getregval(REGISTER_ES), getregval(REGISTER_DI));
                break;
            }
            case OPCODE_STOSW: {
                writeWord(REGISTER_ES, getregval(REGISTER_DI), getregval(REGISTER_AX));

                if (getflag(FLAG_DIRECTION))
                    setregval(REGISTER_DI, getregval(REGISTER_DI) - 2);
                else
                    setregval(REGISTER_DI, getregval(REGISTER_DI) + 2);

                regdump();
                FN_ILOG("STOSW (%04X into %04X:%04X)\n", getregval(REGISTER_AX), getregval(REGISTER_ES), getregval(REGISTER_DI));
                break;
            }
            case OPCODE_REP: {
                uint8_t next = readByte(SEG, IP);
                IP++;

                int reps = 0;
                while (running && getregval(REGISTER_CX)>0) { // will repeat until CX = 0
                    switch (next) {
                        case OPCODE_MOVSW: {
                            uint16_t d = readWord(SEG_OVERRIDE, getregval(REGISTER_SI));
                            writeWord(REGISTER_ES, getregval(REGISTER_DI), d);

                            if (getflag(FLAG_DIRECTION)) {
                                setregval(REGISTER_SI, getregval(REGISTER_SI) - 2);
                                setregval(REGISTER_DI, getregval(REGISTER_DI) - 2);
                            } else {
                                setregval(REGISTER_SI, getregval(REGISTER_SI) + 2);
                                setregval(REGISTER_DI, getregval(REGISTER_DI) + 2);
                            }

                            break;
                        }
                        case OPCODE_STOSB: {
                            writeByte(REGISTER_ES, getregval(REGISTER_DI), getregval(REGISTER_AL));

                            if (getflag(FLAG_DIRECTION))
                                setregval(REGISTER_DI, getregval(REGISTER_DI) - 1);
                            else
                                setregval(REGISTER_DI, getregval(REGISTER_DI) + 1);

                            break;
                        }
                        case OPCODE_STOSW: {
                            writeWord(REGISTER_ES, getregval(REGISTER_DI), getregval(REGISTER_AX));

                            if (getflag(FLAG_DIRECTION))
                                setregval(REGISTER_DI, getregval(REGISTER_DI) - 2);
                            else
                                setregval(REGISTER_DI, getregval(REGISTER_DI) + 2);

                            break;
                        }
                        case OPCODE_CMPSB: {
                            sint8_t d = readByte(SEG_OVERRIDE, getregval(REGISTER_SI));
                            sint8_t d2 = readByte(REGISTER_ES, getregval(REGISTER_DI));

                            if (getflag(FLAG_DIRECTION)) {
                                setregval(REGISTER_SI, getregval(REGISTER_SI) - 1);
                                setregval(REGISTER_DI, getregval(REGISTER_DI) - 1);
                            } else {
                                setregval(REGISTER_SI, getregval(REGISTER_SI) + 1);
                                setregval(REGISTER_DI, getregval(REGISTER_DI) + 1);
                            }

                            if (d - d2 == 0)
                                setflag(FLAG_ZERO);
                            else
                                clearflag(FLAG_ZERO);

                            break;
                        }
                        default:
                            regdump_force();
                            printf("ERROR: Unimplemented REP opcode %02X\n", next);
                            running = false;
                            break;
                    }

                    setregval(REGISTER_CX, getregval(REGISTER_CX) - 1);
                    reps++;
                }

                regdump();
                FN_ILOG("REP (%d reps)\n",reps);

                break;
            }
           /* case 0xC7: {
                if (readByte(SEG, IP) == 0x06) {
                    // honestly, can't be arsed to write a real handler for this on a saturday night. will do later.
                    IP++;

                    uint16_t addr1 = readWord(SEG, IP);
                    IP+=2;
                    uint16_t val = readWord(SEG, IP);
                    IP+=2;

                    regdump();
                    FN_ILOG("MOV WORD [0x%04X], 0x%04X\n", addr1, val);

                    writeWord(SEG_OVERRIDE, addr1, val);

                    break;
                }

                break;
            }*/
            case 0x2E: { //TODO: research how this insn is actually encoded etc
                if (readByte(SEG, IP) == 0xA2) {
                    IP++;

                    uint16_t targ = readWord(SEG, IP);
                    IP+=2;

                    writeByte(REGISTER_CS, targ, getregval(REGISTER_AL));

                    regdump();
                    FN_ILOG("MOV [CS:0x%04X], AL\n", targ);

                } else if (readByte(SEG, IP) == 0x2E) {
                    IP++;

                    IP+=2; // a superflous word here?

                    uint16_t targ = readWord(SEG, IP);
                    IP+=2;

                    setregval(REGISTER_DL, readByte(REGISTER_CS, targ));

                    regdump();
                    FN_ILOG("MOV DL, [CS:0x%04X]\n", targ);
                }

                break;
            }
            case OPCODE_JMP_EA_FAR: {
                uint16_t taddr = readWord(SEG, IP);
                IP+=2;
                uint16_t tseg = readWord(SEG, IP);
                IP+=2;

                setregval(REGISTER_CS, tseg);
                jump(taddr);

                regdump();
                FN_ILOG("JMP FAR %04X:%04X\n",tseg,taddr);

                break;
            }
            case OPCODE_IN_AX:
            case OPCODE_IN_AL: { //TODO: make IO ports do things
                uint8_t port = readByte(SEG, IP);
                IP++;

                regdump();
                FN_ILOG("IN AL, 0x%02X\n",port);

                break;
            }
            case OPCODE_OUT_AX:
            case OPCODE_OUT_AL: {
                uint8_t port = readByte(SEG, IP);
                IP++;

                regdump();
                FN_ILOG("OUT 0x%02X, AL\n",port);

                break;
            }
            case OPCODE_AND_AL_IM8: {
                uint8_t term = readByte(SEG, IP);
                IP++;

                setregval(REGISTER_AL, getregval(REGISTER_AL) & term);

                if (getregval(REGISTER_AL) == 0)
                    setflag(FLAG_ZERO);
                else
                    clearflag(FLAG_ZERO);

                regdump();
                FN_ILOG("AND AL, %02X [8im ipc24]\n", term);

                break;
            }
            case OPCODE_AND_AX_IM16: {
                uint16_t term = readWord(SEG, IP);
                IP++;

                setregval(REGISTER_AX, getregval(REGISTER_AX) & term);

                if (getregval(REGISTER_AX) == 0)
                    setflag(FLAG_ZERO);
                else
                    clearflag(FLAG_ZERO);

                regdump();
                FN_ILOG("AND AX, %04X [16im ipc25]\n", term);

                break;
            }
            case OPCODE_LOOP_REL8: {
                sint8_t targ = readByte(SEG, IP);
                IP++;

                uint8_t shouldjump = (getregval(REGISTER_CX)!=0);

                if (shouldjump) {
                    jump(IP + targ);
                    setregval(REGISTER_CX, getregval(REGISTER_CX) - 1);
                }

                regdump();
                FN_ILOG("LOOP 0x%02X [rel8, %s]\n",IP, shouldjump?"jumped":"not jumped");
                break;
            }
            case OPCODE_PUSHA: {
                pushw(getregval(REGISTER_AX));
                pushw(getregval(REGISTER_CX));
                pushw(getregval(REGISTER_DX));
                pushw(getregval(REGISTER_BX));

                pushw(getregval(REGISTER_SP));
                pushw(getregval(REGISTER_BP));

                pushw(getregval(REGISTER_SI));
                pushw(getregval(REGISTER_DI));

                regdump();
                FN_ILOG("PUSHA\n");

                break;
            }
            case OPCODE_POPA: {
                setregval(REGISTER_DI, popw());
                setregval(REGISTER_SI, popw());

                setregval(REGISTER_BP, popw());
                setregval(REGISTER_SP, popw());

                setregval(REGISTER_BX, popw());
                setregval(REGISTER_DX, popw());
                setregval(REGISTER_CX, popw());
                setregval(REGISTER_AX, popw());

                regdump();
                FN_ILOG("POPA\n");

                break;
            }
            case OPCODE_JG_7F_8REL: { // JG 8 BIT RELATIVE
                sint8_t t = readByte(SEG, IP);
                IP++;

                if (lastcmp > 0)
                    jump(IP + t);

                regdump();
                FN_ILOG("JG 0x%04X %s [8rel]\n",IP,(lastcmp <= 0)?"[not jumped]":"[jumped]");
                break;
            }
            case OPCODE_JC_72_8REL: { // JC 8 BIT RELATIVE
                sint8_t t = readByte(SEG, IP);
                IP++;

                if (getflag(FLAG_CARRY))
                    jump(IP + t);

                regdump();
                FN_ILOG("JC 0x%04X %s [8rel]\n",IP,getflag(FLAG_CARRY)?"[jumped]":"[not jumped]");
                break;
            }
            case OPCODE_JNG_7E_8REL: { // JNG 8 BIT RELATIVE
                sint8_t t = readByte(SEG, IP);
                IP++;

                if (lastcmp < 0)
                    jump(IP + t);

                regdump();
                FN_ILOG("JNG 0x%04X %s [8rel]\n",IP,getflag(FLAG_ZERO)?"[not jumped]":"[jumped]");
                break;
            }
            case OPCODE_SHR_D0_RM8: {
                mrfield mr = parsemr(IP, ipc);
                if (mr.reg == 5) {
                    IP++;
                    if (mr.mod!=0b11) {
                        regdump();
                        FN_ELOG("Unimplemented SHR memory access mode! [ipcD0]\n");
                        running = false;
                        break;
                    }

                    uint8_t reg = mr.rm;
                    uint8_t start = getregval(reg);

                    uint8_t end = (start>>1);
                    uint8_t carry = start&1;

                    if (carry)
                        setflag(FLAG_CARRY);
                    else
                        clearflag(FLAG_CARRY);

                    setregval(reg, end);

                    regdump();
                    FN_ILOG("SHR %s[0x%02X => 0x%02X], 1 [carry=%d]\n", getregname(reg), start, end, carry);
                }

                break;
            }
            case OPCODE_SHRBYCL: {
                mrfield mr = parsemr(IP, ipc);
                if (mr.reg == 5) {
                    IP++;
                    if (mr.mod!=0b11) {
                        regdump();
                        FN_ELOG("Unimplemented SHR memory access mode! [ipcD0]\n");
                        running = false;
                        break;
                    }

                    uint8_t reg = mr.rm;
                    uint8_t start = getregval(reg);

                    uint8_t sha = getregval(REGISTER_CL);

                    uint8_t end = (start>>sha);
                    uint8_t carry = start&(1<<sha);

                    if (carry)
                        setflag(FLAG_CARRY);
                    else
                        clearflag(FLAG_CARRY);

                    setregval(reg, end);

                    regdump();
                    FN_ILOG("SHR %s[0x%02X => 0x%02X], CL [carry=%d]\n", getregname(reg), start, end, carry);
                }

                break;
            }
            case OPCODE_JNC_73_8REL: { // JNC 8 BIT RELATIVE
                sint8_t t = readByte(SEG, IP);
                IP++;

                if (getflag(FLAG_CARRY) == 0)
                    jump(IP + t);

                regdump();
                FN_ILOG("JNC 0x%04X %s [8rel]\n",IP,getflag(FLAG_ZERO)?"[not jumped]":"[jumped]");
                break;
            }
            case OPCODE_TEST_AL: {
                uint8_t mask = readByte(SEG, IP);
                IP++;

                clearflag(FLAG_OVERFLOW);
                clearflag(FLAG_CARRY);

                uint8_t al = getregval(REGISTER_AL);

                uint8_t res = al&mask;

                if (res&0b1000000)
                    setflag(FLAG_SIGN);
                else
                    clearflag(FLAG_SIGN);

                if (res)
                    setflag(FLAG_ZERO);
                else
                    clearflag(FLAG_ZERO);

                regdump();
                FN_ILOG("TEST AL, %02X\n",mask);
                break;
            }
            case OPCODE_SUB_29: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                uint8_t regid1 = mr.reg + (mr.w<<3);
                uint8_t regid2 = mr.rm + (mr.w<<3);

                switch (mr.mod) {
                    case MR_RM1:
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented SUB memory access mode %d [ipc89/8B]\n",mr.mod);
                        running = false;
                        break;
                    case MR_2REG:
                        if (getregval(regid2) - getregval(regid1)==0)
                            setflag(FLAG_ZERO);

                        setregval(regid2, getregval(regid2) - getregval(regid1));

                        regdump();
                        FN_ILOG("SUB %s, %s [subgr m3]\n", getregname(regid2), getregname(regid1));
                }

                break;
            }
            case OPCODE_XOR_AL_8IM: {
                uint8_t term = readByte(SEG, IP);
                IP++;

                setregval(REGISTER_AL, getregval(REGISTER_AL) ^ term);

                regdump();
                FN_ILOG("XOR AL, %02X\n", term);

                break;
            }
            case MOV_MEM_AX_16AM: {
                uint16_t addr = readWord(SEG, IP);
                IP+=2;
                writeWord(REGISTER_DS, addr, getregval(REGISTER_AX));

                regdump();
                FN_ILOG("MOV [%04X], AX [opcA3]\n", addr);

                break;
            }
            case OPCODE_ADD_03: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                // if d is 1, move rmw to sr, else sr to rmw

                mr.w = 1;

                uint8_t regid1 = (mr.reg + (mr.w << 3));

                uint16_t loc;

                switch (mr.mod) {
                    case MR_RM1:
                        if (mr.rm == 0b110) { // Drc't addition
                            loc = readWord(SEG, IP);
                            IP+=2;

                            setregval(regid1, getregval(regid1) + readWord(SEG_OVERRIDE, loc));

                            regdump();
                            FN_ILOG("ADD %s, [0x%04X] [addgr m0 drc't]\n", getregname(regid1), loc);
                            break;
                        }
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16:
                    case MR_2REG:
                        regdump_force();
                        FN_ELOG("FIXME: Unimplemented ADD memory access mode %d [ipc03]\n",mr.mod);
                        running = false;
                        break;
                }

                break;
            }
            case XCHG_AX_CX:
            case XCHG_AX_DX:
            case XCHG_AX_BX:
            case XCHG_AX_SI:
            case XCHG_AX_DI:
            case XCHG_AX_BP:
            case XCHG_AX_SP: {
                uint8_t regid = (ipc - 0x90) + 8;

                uint16_t ov = getregval(regid);
                setregval(regid, getregval(REGISTER_AX));
                setregval(REGISTER_AX, ov);

                regdump();
                FN_ILOG("XCHG AX, %s\n",getregname(regid));

                break;
            }
            case OPCODE_DEC_SI: {
                setregval(REGISTER_SI, getregval(REGISTER_SI) - 1);

                if (getregval(REGISTER_SI)  == 0)
                    setflag(FLAG_ZERO);
                else
                    clearflag(FLAG_ZERO);

                regdump();
                FN_ILOG("DEC SI\n");
                break;
            }
            case OPCODE_XOR_30_MR8_REV:
            case OPCODE_XOR_32_MR8: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                switch (mr.mod) {
                    case MR_RM1:
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16: {
                        FN_ELOG("ERROR: Unimplented XOR memory access mode %d\n", mr.mod);
                        running = false;
                        break;
                    }
                    case MR_2REG: {
                        uint8_t regid1, regid2;
                        if (ipc == OPCODE_XOR_30_MR8_REV) {
                            regid1 = mr.rm + (mr.w << 3);
                            regid2 = mr.reg + (mr.w << 3);
                        } else {
                            regid1 = mr.reg + (mr.w << 3);
                            regid2 = mr.rm + (mr.w << 3);
                        }

                        setregval(regid1, getregval(regid1) ^ getregval(regid2));

                        regdump();
                        FN_ILOG("XOR %s, %s [ipc%02X]\n", getregname(regid1), getregname(regid2), ipc);
                    }
                }

                break;
            }
            case OPCODE_STC: {
                setflag(FLAG_CARRY);

                regdump();
                FN_ILOG("STC\n");
                break;
            }
            case OPCODE_XOR_33_MR16: {
                mrfield mr = parsemr(IP, ipc);
                IP++;

                switch (mr.mod) {
                    case MR_RM1:
                    case MR_RM2_DISP8:
                    case MR_RM2_DISP16: {
                        FN_ELOG("ERROR: Unimplented XOR memory access mode %d\n", mr.mod);
                        running = false;
                        break;
                    }
                    case MR_2REG: {
                        uint8_t regid1, regid2;
                        if (ipc == OPCODE_XOR_30_MR8_REV) {
                            regid1 = mr.rm + (mr.w << 3);
                            regid2 = mr.reg + (mr.w << 3);
                        } else {
                            regid1 = mr.reg + (mr.w << 3);
                            regid2 = mr.rm + (mr.w << 3);
                        }

                        setregval(regid1, getregval(regid1) ^ getregval(regid2));

                        regdump();
                        FN_ILOG("XOR %s, %s [ipc%02X]\n", getregname(regid1), getregname(regid2), ipc);
                    }
                }

                break;
            }
            default: {
                SEG_OVERRIDE = REGISTER_DS;
                if (ipc == 0x26) { // handle ES override prefix
                    printf("ES segment override applied for one cycle.\n");
                    SEG_OVERRIDE = REGISTER_ES;
                } else {
                    regdump();
                    char l1b[512];
                    strcpy(l1b, lastinsn);
                    sprintf(lastinsn, "Illegal instruction at 0x%04X:0x%04X [opcode = %02X], last insn [%s]\n",
                            getregval(SEG), IP - 1, ipc, l1b);
                    printf("%s", lastinsn);
                    running = false;
                    break;
                }
            }
        }

        for (char & i : lastinsn) {
            if (i == '\n') {
                i = 0x20;
            }
        }

        uint32_t stackptr = segcalc(getregval(REGISTER_SS), getregval(REGISTER_SP));
        if (stackptr > VIRTUAL_MEMORY_SIZE) {
            char l1b[512];
            strcpy(l1b, lastinsn);
            FN_ELOG("[%s]: STACK POINTER EXCEEDS VIRTUAL MEMORY SIZE!\n",l1b);
            running = false;
            break;
        }

        if (running) {
#ifdef SINGLE_STEP
            if ((IP >= 0x200 || IP <= 0x100)) { // disable singlestep in non-writable memory (i.e the ROM area)
                waitForKey();
            }
#endif

#ifdef INSN_DELAY_MS
            std::this_thread::sleep_for(std::chrono::milliseconds(INSN_DELAY_MS));
#else
#ifdef INSN_DELAY
            for (uint32_t i=0;i<(1<<INSN_DELAY);i++) {
                __asm__ __volatile__ ("nop"); // shhh
            }
#endif
#endif

            cycles++;
        }
    }

    printf("Halted\n");

    printf("Press a key to end\n");
    getchar();

    killvideo();
    free(virt_memory);
    return 0;
}
