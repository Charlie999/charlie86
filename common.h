#ifndef CHARLIE86_COMMON_H
#define CHARLIE86_COMMON_H

typedef std::pair<std::pair<uint16_t, uint16_t>,std::pair<uint16_t, uint16_t>> callstack_entry_t;
#define CALLSTACK_ENTRY_BLANK std::pair<uint16_t, uint16_t>(0,0), std::pair<uint16_t, uint16_t>(0,0)
#define CALLSTACK_ENTRY_INIT(org, dest) callstack_entry_t(dest, org)

class callstack_t : public std::stack<callstack_entry_t> {
public:
    using std::stack<callstack_entry_t>::c; // expose the container
};

void setregval(uint8_t regid, uint16_t val);
uint16_t getregval(uint8_t regid);
uint8_t * getmemory();
uint16_t getip();
uint16_t* getregs();
uint16_t getflags();
int getrunning();
char* getlastinsn();
char* getlastinsn2();
char* getlastinsn3();
char* getlastinsn4();
uint32_t getstackptr();
callstack_t getcallstack();
unsigned long long getcycles();

#endif //CHARLIE86_COMMON_H
