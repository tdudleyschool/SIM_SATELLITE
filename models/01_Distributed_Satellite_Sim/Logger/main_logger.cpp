#include "Logger.hh"

int main() {
    LoggerActor logger("127.0.0.1", 9000);
    logger.start();

    // Run indefinitely actually for 24 hours
    std::this_thread::sleep_for(std::chrono::hours(24)); // or while(true)
    logger.stop();

    return 0;
}