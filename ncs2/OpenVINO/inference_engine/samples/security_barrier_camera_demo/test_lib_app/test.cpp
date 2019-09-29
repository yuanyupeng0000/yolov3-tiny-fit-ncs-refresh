#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include "intel_dldt.h"

int main(int argc, char *argv[]){
    std::vector<LPObject> licensePlates;
    std::string sInput = std::string(argv[1]);
    std::cout << sInput << std::endl;
    main_process(argc, argv, sInput,licensePlates);
    for (auto && lp : licensePlates) {
    std::cout << lp.text << std::endl;
            }
    return 0;
}
