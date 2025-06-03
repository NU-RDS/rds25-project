#ifndef TEENSYID_HPP
#define TEENSYID_HPP

int* nodeID(int input) {
    static int result[2]; 
    
    switch(input) {
        case 0:
            result[0] = 0;
            result[1] = 1;
            break;
        case 1:
            result[0] = 2;
            result[1] = 3;
            break;
        case 2:
            result[0] = 4;
            result[1] = 4;
            break;
        case 3:
            result[0] = 5;
            result[1] = 6;
            break;
        default:
            result[0] = -1;
            result[1] = -1;
            break;
    }
    
    return result;
}

#endif