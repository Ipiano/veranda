#define CATCH_CONFIG_RUNNER
#include "catch2/catch.hpp"
#include <QCoreApplication>
#include <QTimer>

int main(int argc, char* argv[])
{
    // Create QCoreApplication instance for Qt's signal/slot mechanism
    QCoreApplication app(argc, argv);

    int result = 0;

    // Run Catch tests within Qt's event loop using a single-shot timer
    QTimer::singleShot(0, [&]() {
        // Run Catch session with the command line arguments
        result = Catch::Session().run(argc, argv);

        // Exit the application after tests complete
        QCoreApplication::exit(result);
    });

    // Start Qt event loop
    app.exec();

    return result;
}