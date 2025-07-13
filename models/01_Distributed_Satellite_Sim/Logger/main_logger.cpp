// main_logger.cpp
#include "Logger.hh"
#include <iostream>

int main() {
    LoggerActor logger("127.0.0.1", 9000);
    logger.start();

    std::cout << "[Main] Logger running. Press Enter to stop...\n";
    std::cin.get();

    logger.stop();
    std::cout << "[Main] Logger stopped.\n";
    return 0;
}
