#include <settings.hpp>

std::ofstream &Printer::get() const {
    static std::ofstream output;
    return output;
}
