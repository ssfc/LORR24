#include <Objects/Environment/operations.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>
#include <Objects/Environment/info.hpp>

#include <fstream>

bool verify_operation(const Operation &op) {
    // не нужно, чтобы на конце операции были повороты
    // грамотный подсчет веса операции это увидит
    for (int i = op.size() - 1; i >= 0 && op[i] != Action::FW; i--) {
        if (op[i] == Action::CR || op[i] == Action::CCR) {
            return false;
        }
    }
    // операции вида CWR плохие, потому что мы повернулись, прошло пашу шагов и мы повернулись обратно
    for (int i = 0; i < op.size(); i++) {
        for (int j = i + 1; j < op.size() && op[j] != Action::FW; j++) {
            if (op[i] == Action::CR && op[j] == Action::CCR) {
                return false;
            }
            if (op[i] == Action::CCR && op[j] == Action::CR) {
                return false;
            }
        }
    }
    // операции вида CWC плохие, потому что мы повернулись, подождали и повернулись еще раз
    FAILED_ASSERT("TODO");

    // операции вида RRR плохие, потому что мы 3 раза крутимся, когда могли 1: CWW
    for (int i = 0; i < op.size(); i++) {
        for (int j = i + 1; j < op.size() && op[j] != Action::FW; j++) {
            for (int k = j + 1; k < op.size() && op[k] != Action::FW; k++) {
                if ((op[i] == Action::CR || op[i] == Action::CCR) && op[i] == op[j] && op[j] == op[k]) {
                    return false;
                }
            }
        }
    }
    return true;
}

void OperationsGenerator::generate(Operation &op, uint32_t i) {
    if (i == DEPTH) {
        if (verify_operation(op)) {
            pool.push_back(op);
        }
    } else {
        for (int32_t action = 0; action < 4; action++) {
            op[i] = static_cast<Action>(action);
            generate(op, i + 1);
        }
    }
}

std::vector<Operation> OperationsGenerator::get() {
    //Operation op;
    //generate(op, 0);

    // read pool
    {
        std::vector<std::string> operations_pool_strs = {
                // вообще тут получилось 3951->4108, но там немного стремная логика
                // 3951 and 6080
                "17 WWW RRF RWF CWF WWF RFW CFW WFW RFF CFF FWW WFF FRF FCF FWF FFW FFF",

                // 3831 and 6373
                "48 WWWW RRWF RWWF CWWF WWWF RRFW RWFW CWFW WWFW RRFF RFWW RWFF CFWW CWFF WFWW WWFF RFRF RFCF CFRF CFCF RFWF CFWF WFRF WFCF FWWW WFWF FRRF RFFW CFFW FRWF FCWF FWWF WFFW RFFF CFFF FRFW FCFW FWFW WFFF FRFF FCFF FFWW FWFF FFRF FFCF FFWF FFFW FFFF",

                // 3670 and 6153
                "136 WWWWW CCWWF RWWWF CWWWF WWWWF RRWFW RWWFW CWWFW WWWFW RRFWW RRWFF RWFWW RWWFF CWFWW CWWFF RRFRF RRFCF WWFWW WWWFF RRFWF RWFRF RWFCF CWFRF CWFCF RFWWW RWFWF CFWWW CWFWF WWFRF WWFCF WFWWW WWFWF RFRRF CFRRF RRFFW RFRWF RFCWF CFRWF CFCWF RFWWF RWFFW CFWWF CWFFW WFRRF WFRWF WFCWF FWWWW WFWWF WWFFW RRFFF RFRFW RFCFW CFRFW CFCFW FRRWF RFWFW RWFFF CFWFW CWFFF FRWWF FCWWF WFRFW WFCFW FWWWF WFWFW WWFFF RFRFF RFCFF CFRFF CFCFF FRRFW RFFWW RFWFF CFFWW CFWFF FRWFW FCWFW WFRFF WFCFF FWWFW WFFWW WFWFF FRRFF RFFRF RFFCF CFFRF CFFCF RFFWF CFFWF FRFWW FRWFF FCFWW FCWFF WFFRF WFFCF FWFWW FWWFF WFFWF FRFRF FRFCF FCFRF FCFCF RFFFW CFFFW FRFWF FCFWF FWFRF FWFCF FFWWW FWFWF WFFFW FFRRF RFFFF CFFFF FRFFW FCFFW FFRWF FFCWF FFWWF FWFFW WFFFF FRFFF FCFFF FFRFW FFCFW FFWFW FWFFF FFRFF FFCFF FFFWW FFWFF FFFRF FFFCF FFFWF FFFFW FFFFF",
        };
        std::stringstream input(operations_pool_strs.at(get_epibt_operation_depth() - 3));
        uint32_t num;
        input >> num;
        for (uint32_t i = 0; i < num; i++) {
            std::string str;
            input >> str;
            Operation op;
            str.resize(DEPTH, 'W');
            //ASSERT(str.size() == op.size(), "does not match sizes: >" + str + "<, " + std::to_string(op.size()));
            std::stringstream ss(str);
            ss >> op;
            pool.push_back(op);
        }
    }

    /*auto get_operation_weight = [&](Operation op) {
        double s = 0;
        for (uint32_t d = 0; d < op.size(); d++) {
            s += (op[d] == Action::FW) * (op.size() - d) * 10;
            s -= (op[d] == Action::W) * (op.size() - d) * 1;
            s -= (op[d] == Action::CR) * (op.size() - d) * 2;
            s -= (op[d] == Action::CCR) * (op.size() - d) * 2;
        }
        return s;
    };
    std::stable_sort(pool.begin(), pool.end(), [&](auto lhs, auto rhs) {
        return get_operation_weight(lhs) < get_operation_weight(rhs);
    });*/

    // add WWW
    /*{
        Operation op;
        for (uint32_t i = 0; i < op.size(); i++) {
            op[i] = Action::W;
        }
        auto it = std::find(pool.begin(), pool.end(), op);
        if (it != pool.end()) {
            pool.erase(it);
        }
        pool.insert(pool.begin(), op);
    }*/

    std::vector<Operation> result = pool;

    /*std::set<std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>>> visited;
    for (auto operation: pool) {
        std::array<std::pair<uint32_t, uint32_t>, DEPTH> positions{};
        Position p;
        std::set<std::pair<uint32_t, uint32_t>> visited_poses;
        visited_poses.insert({p.get_x(), p.get_y()});
        for (uint32_t d = 0; d < DEPTH; d++) {
            p = p.simulate_action(operation[d]);
            positions[d] = {p.get_x(), p.get_y()};
            visited_poses.insert(positions[d]);
        }
        std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>> kek = {p.get_dir(), positions};
        if (!visited.count(kek)) {
            visited.insert(kek);
            result.push_back(operation);
        }
    }*/

    PRINT(
            Printer() << "Operations:\n"
                      << result.size() << ' ';
            for (auto operation
                 : result) {
                Printer() << operation << ' ';
            };
            Printer() << '\n';);
    //_exit(100);
    return result;
}

std::vector<Operation> &get_operations() {
    static std::vector<Operation> operations;
    return operations;
}

static inline std::vector<uint32_t> operation_depth;

uint32_t get_operation_depth(uint32_t index) {
    ASSERT(0 <= index && index < operation_depth.size(), "invalid index");
    ASSERT(operation_depth.size() == get_operations().size(), "unmatch sizes");
    return operation_depth[index];
}

std::vector<uint32_t> &get_operations_ids(uint32_t d) {
    static std::array<std::vector<uint32_t>, DEPTH + 1> data;
    ASSERT(3 <= d && d <= 5, "invalid d");
    return data[d];
}

void init_operations() {
    get_operations() = OperationsGenerator().get();

    auto get_operation_depth = [&](const Operation &op) {
        uint32_t d = op.size();
        for (; d > 0 && op[d - 1] == Action::W; d--) {
        }
        return d;
    };

    operation_depth.resize(get_operations().size());
    for (uint32_t i = 0; i < get_operations().size(); i++) {
        uint32_t d = get_operation_depth(get_operations()[i]);
        if (d <= 3) {
            operation_depth[i] = 3;
            get_operations_ids(3).push_back(i);
        } else if (d == 4) {
            operation_depth[i] = 4;
            get_operations_ids(4).push_back(i);
        } else if (d == 5) {
            operation_depth[i] = 5;
            get_operations_ids(5).push_back(i);
        } else {
            FAILED_ASSERT("unexpected depth");
        }
    }
}

std::ostream &operator<<(std::ostream &output, const Operation &op) {
    for (auto op: op) {
        if (op == Action::W) {
            output << 'W';
        } else if (op == Action::FW) {
            output << 'F';
        } else if (op == Action::CCR) {
            output << 'C';
        } else if (op == Action::CR) {
            output << 'R';
        } else if (op == Action::NA) {
            output << 'N';
        }
    }
    return output;
}

std::istream &operator>>(std::istream &input, Operation &op) {
    for (auto &op: op) {
        char c;
        input >> c;
        if (c == 'W') {
            op = Action::W;
        } else if (c == 'F') {
            op = Action::FW;
        } else if (c == 'C') {
            op = Action::CCR;
        } else if (c == 'R') {
            op = Action::CR;
        } else if (c == 'N') {
            op = Action::NA;
        } else {
            ASSERT(false, "invalid action");
        }
    }
    return input;
}
