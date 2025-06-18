#include <core/ember-app.hh>
#include <rich-log/log.hh>

int main(int argc, char** args)
{
    // Create and run EMBER app
    mk::EmberApp ember_app;
    ember_app.run(argc, args);
    
    return 0;
}