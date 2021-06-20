#ifndef CHARLIE86_FLAGS_H
#define CHARLIE86_FLAGS_H

#define FLAG_CARRY      0b1
// 0b10 reserved
#define FLAG_PARITY     0b100
// 0b1000 reserved
#define FLAG_ADJUST     0b10000
// 0b100000 reserved
#define FLAG_ZERO       0b1000000
#define FLAG_SIGN       0b10000000
#define FLAG_TRAP       0b100000000
#define FLAG_INT        0b1000000000
#define FLAG_DIRECTION  0b10000000000
#define FLAG_OVERFLOW   0b100000000000

// rest reserved

#endif //CHARLIE86_FLAGS_H
