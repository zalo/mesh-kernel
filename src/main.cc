#include <core/kernel-app.hh>

int main(int argc, char** args)
{
    mk::KernelApp kernel_app(rlog::verbosity::Trace);
    kernel_app.run(argc, args);
    return 0;
}
