#include <algorithm>
#include <cstdint>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <stdexcept>
#include <iomanip>
#include <string>
#include <sstream>

#include <iostream>

using namespace std;

enum class DumpMode
{
    NONE,
    USED,
    ALL
};

static DumpMode parse_dump_mode(const string &s)
{
    if (s == "none" || s == "None")
        return DumpMode::NONE;
    if (s == "used" || s == "Used")
        return DumpMode::USED;
    if (s == "all" || s == "All")
        return DumpMode::ALL;
    throw runtime_error("Invalid dump mode: " + s);
}

static void print_help()
{
    std::cout << "MRV32 Instruction Set Simulator\n"
                 "Usage:\n"
                 "  mrv_iss [options] <program.elf>\n\n"
                 "Options:\n"
                 "  -h, --help\n"
                 "      Show this help message and exit\n\n"
                 "  --max-steps=<n>\n"
                 "      Maximum number of instructions to execute (default: 1000)\n\n"
                 "  --trace=(true|false)\n"
                 "      Enable per-instruction trace output (default: false)\n\n"
                 "  --show-memory=(none|used|all)\n"
                 "      Control memory dump behavior\n"
                 "        none   : do not display memory\n"
                 "        used   : display memory written in the current step\n"
                 "        all    : display full memory range each step\n\n"
                 "  --mem-addr-base=<addr>\n"
                 "      Base address for --show-memory=all (default: 0x00000000)\n\n"
                 "  --mem-addr-end=<addr>\n"
                 "      End address for --show-memory=all (default: 0x000000FF)\n\n"
                 "  --show-register-file=(none|used|all)\n"
                 "      Control register file dump behavior\n"
                 "  --show-stats=(true|false)\n"
                 "      Dump several statistics at the end of the simulation\n"
                 "  --dump-instrs\n\n"
                 "      Show instructions in the ELF in order\n"
                 "Examples:\n"
                 "  mrv_iss test.elf\n"
                 "  mrv_iss --trace=true --max-steps=10000 test.elf\n"
                 "  mrv_iss --trace=true --show-memory=used test.elf\n";
}

struct Config
{
    string elf_path;

    uint64_t max_steps = 1000; // default
    bool trace = false;
    bool dump_instrs = false;
    bool show_stats = true;

    DumpMode show_memory = DumpMode::NONE;
    DumpMode show_rf = DumpMode::NONE;

    uint32_t mem_base = 0x00000000; // default range for ALL
    uint32_t mem_end = 0x000000FF;
};

static bool starts_with(const string &s, const string &p)
{
    return s.rfind(p, 0) == 0;
}

Config parse_args(int argc, char **argv)
{
    Config cfg;

    for (int i = 1; i < argc; i++)
    {
        string a(argv[i]);

        if (a == "--trace=true")
        {
            cfg.trace = true;
        }
        else if (a == "--trace=false")
        {
            cfg.trace = false;
        }
        else if (a == "--trace")
        {
            cfg.trace = true;
        }
        else if (a == "--show-stats" || a == "--show-stats=ture")
        {
            cfg.trace = true;
        }
        else if (a == "--show-stats=false")
        {
            cfg.show_stats = false;
        }
        else if (starts_with(a, "--max-steps="))
        {
            cfg.max_steps = stoull(a.substr(12));
        }
        else if (starts_with(a, "--show-memory="))
        {
            cfg.show_memory = parse_dump_mode(a.substr(14));
        }
        else if (starts_with(a, "--show-register-file="))
        {
            cfg.show_rf = parse_dump_mode(a.substr(21));
        }
        else if (starts_with(a, "--mem-addr-base="))
        {
            cfg.mem_base = stoul(a.substr(16), nullptr, 0);
        }
        else if (starts_with(a, "--mem-addr-end="))
        {
            cfg.mem_end = stoul(a.substr(15), nullptr, 0);
        }
        else if (starts_with(a, "--dump-instrs"))
        {
            cfg.dump_instrs = true;
        }
        else if (a == "--help" || a == "-h")
        {
            print_help();
            exit(0);
        }
        else if (a[0] == '-')
        {
            throw runtime_error("Unknown option: " + a);
        }
        else
        {
            // positional argument = ELF path
            if (!cfg.elf_path.empty())
                throw runtime_error("Multiple ELF files specified");
            cfg.elf_path = a;
        }
    }

    if (cfg.elf_path.empty())
        throw runtime_error("No ELF file specified");

    return cfg;
}

static const char *regname(uint32_t r)
{
    static const char *names[32] = {
        "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7",
        "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15",
        "x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23",
        "x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31"};
    return names[r & 31];
}

static std::string hex32(uint32_t v)
{
    std::ostringstream oss;
    oss << "0x" << std::hex << std::setw(8) << std::setfill('0') << v;
    return oss.str();
}

/************************************************* REGISTER FILE *************************************************/

class RegisterFile
{
public:
    RegisterFile(int size);
    ~RegisterFile();

    int getSize() const;
    uint32_t getValue(int index) const;
    void writeValue(int index, uint32_t value);

    void begin_step() { step_writes.clear(); }
    const auto &get_step_writes() const { return step_writes; }
    const auto &get_used_regs() const { return used_regs; }

private:
    uint32_t *rf;
    int sz;

    std::unordered_set<int> used_regs;
    std::vector<std::pair<int, uint32_t>> step_writes;
};

RegisterFile::RegisterFile(int size)
{
    // Constructor implementation
    sz = size;
    rf = new uint32_t[size]();
}

RegisterFile::~RegisterFile()
{
    // Destructor implementation
    delete[] rf;
}

int RegisterFile::getSize() const
{
    return sz;
}

void RegisterFile::writeValue(int index, uint32_t value)
{
    if (index == 0)
        return; // x0 hardwired to 0
    if (index < 0 || index >= sz)
    {
        cerr << "Index OOB\n";
        return;
    }
    rf[index] = value;
    used_regs.insert(index);
    step_writes.push_back({index, value});
}

uint32_t RegisterFile::getValue(int index) const
{
    if (index < 0 || index >= sz)
    {
        cerr << "Index out of bounds" << endl;
        return -1; // or handle error appropriately
    }
    return rf[index];
}

/************************************************* MEMORY *************************************************/

class Memory
{
public:
    struct MemWrite
    {
        uint32_t addr;
        uint32_t size;  // 1,2,4
        uint32_t value; // full store value
    };

    void begin_step() { step_writes.clear(); }
    const std::vector<MemWrite> &get_step_writes() const { return step_writes; }

    // "Used memory" tracking (whole run)
    const std::unordered_set<uint32_t> &get_used_bytes() const { return used_bytes; }

    // ---- Writes (LOG ONCE, NO PER-BYTE SPAM) ----
    void write8(uint32_t addr, uint8_t val)
    {
        store8_raw(addr, val);
        step_writes.push_back({addr, 1, (uint32_t)val});
    }

    void write16(uint32_t addr, uint16_t val)
    {
        store8_raw(addr + 0, (uint8_t)(val & 0xFF));
        store8_raw(addr + 1, (uint8_t)((val >> 8) & 0xFF));
        step_writes.push_back({addr, 2, (uint32_t)val});
    }

    void write32(uint32_t addr, uint32_t val)
    {
        store8_raw(addr + 0, (uint8_t)((val >> 0) & 0xFF));
        store8_raw(addr + 1, (uint8_t)((val >> 8) & 0xFF));
        store8_raw(addr + 2, (uint8_t)((val >> 16) & 0xFF));
        store8_raw(addr + 3, (uint8_t)((val >> 24) & 0xFF));
        step_writes.push_back({addr, 4, val});
    }

    // ---- Reads (same as you had) ----
    uint8_t read8(uint32_t addr) const
    {
        auto it = mem.find(addr);
        return (it == mem.end()) ? 0 : it->second;
    }

    int8_t read8s(uint32_t addr) const { return (int8_t)read8(addr); }

    uint16_t read16(uint32_t addr) const
    {
        uint16_t b0 = read8(addr);
        uint16_t b1 = read8(addr + 1);
        return (uint16_t)(b0 | (b1 << 8));
    }

    int16_t read16s(uint32_t addr) const { return (int16_t)read16(addr); }

    uint32_t read32(uint32_t addr) const
    {
        uint32_t b0 = read8(addr);
        uint32_t b1 = read8(addr + 1);
        uint32_t b2 = read8(addr + 2);
        uint32_t b3 = read8(addr + 3);
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
    }

private:
    // base byte write: updates memory + used-bytes set, but DOES NOT log
    void store8_raw(uint32_t addr, uint8_t val)
    {
        mem[addr] = val;
        used_bytes.insert(addr);
    }

    std::unordered_map<uint32_t, uint8_t> mem;

    std::vector<MemWrite> step_writes;       // per-step stores
    std::unordered_set<uint32_t> used_bytes; // all bytes ever written
};

/****************************************************** CPU ******************************************************/

static inline uint32_t get_bits(uint32_t x, int hi, int lo)
{
    return (x >> lo) & ((1u << (hi - lo + 1)) - 1);
}

static inline int32_t sext(uint32_t x, int bits)
{
    // sign-extend x which is 'bits' wide
    uint32_t m = 1u << (bits - 1);
    return (int32_t)((x ^ m) - m);
}

static inline int32_t imm_i(uint32_t inst)
{
    return sext(get_bits(inst, 31, 20), 12);
}

static inline int32_t imm_s(uint32_t inst)
{
    uint32_t imm = (get_bits(inst, 31, 25) << 5) | get_bits(inst, 11, 7);
    return sext(imm, 12);
}

static inline int32_t imm_b(uint32_t inst)
{
    uint32_t imm =
        (get_bits(inst, 31, 31) << 12) |
        (get_bits(inst, 7, 7) << 11) |
        (get_bits(inst, 30, 25) << 5) |
        (get_bits(inst, 11, 8) << 1);
    return sext(imm, 13);
}

static inline int32_t imm_j(uint32_t inst)
{
    uint32_t imm =
        (get_bits(inst, 31, 31) << 20) |
        (get_bits(inst, 19, 12) << 12) |
        (get_bits(inst, 20, 20) << 11) |
        (get_bits(inst, 30, 21) << 1);
    return sext(imm, 21);
}

struct CPU
{
    uint32_t pc = 0;
    RegisterFile rf;

    struct Stats
    {
        uint64_t instret = 0;
        uint64_t loads = 0;
        uint64_t stores = 0;
        uint64_t branches = 0;
        uint64_t taken_branches = 0;
        uint64_t jumps = 0;

        uint64_t bytes_read = 0;
        uint64_t bytes_written = 0;
    };

    CPU() : rf(32) {}
    Stats stats{};

    // returns false if should stop (for now: illegal instruction)
    bool step(Memory &mem, const Config &cfg)
    {
        if (pc & 3)
            throw runtime_error("Misaligned PC");

        uint32_t inst = mem.read32(pc);
        uint32_t old_pc = pc;
        pc += 4;

        uint32_t opcode = get_bits(inst, 6, 0);
        uint32_t rd = get_bits(inst, 11, 7);
        uint32_t funct3 = get_bits(inst, 14, 12);
        uint32_t rs1 = get_bits(inst, 19, 15);
        uint32_t rs2 = get_bits(inst, 24, 20);
        uint32_t funct7 = get_bits(inst, 31, 25);

        auto trace_line = [&](const string &s)
        {
            if (cfg.trace)
            {
                cout << hex << setw(8) << setfill('0') << old_pc
                     << ": " << setw(8) << inst << "  " << s << "\n";
            }
        };

        switch (opcode)
        {
        // I-Type - ALOps
        case 0x13:
        {
            // ADDI
            switch (funct3)
            {
            // ADDI
            case 0x0:
            {

                int32_t imm = imm_i(inst);
                uint32_t v = rf.getValue(rs1) + (uint32_t)imm;
                rf.writeValue(rd, v);
                trace_line("addi " +
                           string(regname(rd)) + ", " +
                           string(regname(rs1)) + ", " +
                           to_string(imm));
                stats.instret++;
                return true;
            }
            case 0x1:
            {
                // SLLI
                if (funct7 == 0x00)
                {
                    uint32_t shamt = get_bits(inst, 24, 20) & 0x1F;
                    uint32_t v = rf.getValue(rs1) << shamt;
                    rf.writeValue(rd, v);
                    trace_line("slli " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               to_string(shamt));
                    stats.instret++;
                    return true;
                }
                break;
            }
            case 0x2:
            {
                // SLTI
                int32_t imm = imm_i(inst);
                uint32_t v = (int32_t)rf.getValue(rs1) < imm ? 1 : 0;
                rf.writeValue(rd, v);
                trace_line("slti " +
                           string(regname(rd)) + ", " +
                           string(regname(rs1)) + ", " +
                           to_string(imm));
                stats.instret++;
                return true;
            }
            case 0x3:
            {
                // SLTIU
                int32_t imm = imm_i(inst);
                uint32_t v = rf.getValue(rs1) < (uint32_t)imm ? 1 : 0;
                rf.writeValue(rd, v);
                trace_line("sltiu " +
                           string(regname(rd)) + ", " +
                           string(regname(rs1)) + ", " +
                           to_string(imm));
                stats.instret++;
                return true;
            }
            case 0x4:
            {
                // XORI
                int32_t imm = imm_i(inst);
                uint32_t v = rf.getValue(rs1) ^ (uint32_t)imm;
                rf.writeValue(rd, v);
                trace_line("xori " +
                           string(regname(rd)) + ", " +
                           string(regname(rs1)) + ", " +
                           to_string(imm));
                stats.instret++;
                return true;
            }
            case 0x5:
            {
                if (funct7 == 0x00)
                {
                    // SRLI
                    uint32_t shamt = get_bits(inst, 24, 20) & 0x1F;
                    uint32_t v = rf.getValue(rs1) >> shamt;
                    rf.writeValue(rd, v);
                    trace_line("srli " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               to_string(shamt));
                    stats.instret++;
                    return true;
                }
                else if (funct7 == 0x20)
                {
                    // SRAI
                    uint32_t shamt = get_bits(inst, 24, 20) & 0x1F;
                    int32_t sv = (int32_t)rf.getValue(rs1);
                    uint32_t v = (uint32_t)(sv >> shamt);
                    rf.writeValue(rd, v);
                    trace_line("srai " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               to_string(shamt));
                    stats.instret++;
                    return true;
                }
                break;
            }
            case 0x6:
            {
                // ORI
                int32_t imm = imm_i(inst);
                uint32_t v = rf.getValue(rs1) | (uint32_t)imm;
                rf.writeValue(rd, v);
                trace_line("ori " +
                           string(regname(rd)) + ", " +
                           string(regname(rs1)) + ", " +
                           to_string(imm));
                stats.instret++;
                return true;
            }
            case 0x7:
            {
                // ANDI
                int32_t imm = imm_i(inst);
                uint32_t v = rf.getValue(rs1) & (uint32_t)imm;
                rf.writeValue(rd, v);
                trace_line("andi " +
                           string(regname(rd)) + ", " +
                           string(regname(rs1)) + ", " +
                           to_string(imm));
                stats.instret++;
                return true;
            }
            default:
                break;
            }
            break;
        }

        // I-Type - Loads
        case 0x03:
        {
            switch (funct3)
            {
            // LB
            case 0x0:
            {
                int32_t imm = imm_i(inst);
                uint32_t addr = rf.getValue(rs1) + (uint32_t)imm;
                int8_t data = mem.read8s(addr);
                rf.writeValue(rd, (int32_t)data);
                trace_line("lb " +
                           string(regname(rd)) + ", " +
                           to_string(imm) + "(" +
                           string(regname(rs1)) + ")");
                stats.bytes_read += 1;
                stats.loads++;
                stats.instret++;
                return true;
            }
            // LH
            case 0x1:
            {
                int32_t imm = imm_i(inst);
                uint32_t addr = rf.getValue(rs1) + (uint32_t)imm;

                if (addr & 1)
                    throw runtime_error("Misaligned LH");

                int16_t data = mem.read16s(addr);
                rf.writeValue(rd, (int32_t)data);
                trace_line("lh " +
                           string(regname(rd)) + ", " +
                           to_string(imm) + "(" +
                           string(regname(rs1)) + ")");
                stats.bytes_read += 2;
                stats.loads++;
                stats.instret++;
                return true;
            }
            // LW
            case 0x2:
            {
                int32_t imm = imm_i(inst);
                uint32_t addr = rf.getValue(rs1) + (uint32_t)imm;

                if (addr & 3)
                    throw runtime_error("Misaligned LW");

                uint32_t data = mem.read32(addr);
                rf.writeValue(rd, data);
                trace_line("lw " +
                           string(regname(rd)) + ", " +
                           to_string(imm) + "(" +
                           string(regname(rs1)) + ")");
                stats.bytes_read += 4;
                stats.loads++;
                stats.instret++;
                return true;
            }
            // LBU
            case 0x4:
            {
                int32_t imm = imm_i(inst);
                uint32_t addr = rf.getValue(rs1) + (uint32_t)imm;
                uint8_t data = mem.read8(addr);
                rf.writeValue(rd, (uint32_t)data);
                trace_line("lbu " +
                           string(regname(rd)) + ", " +
                           to_string(imm) + "(" +
                           string(regname(rs1)) + ")");
                stats.bytes_read += 1;
                stats.loads++;
                stats.instret++;
                return true;
            }
            // LHU
            case 0x5:
            {
                int32_t imm = imm_i(inst);
                uint32_t addr = rf.getValue(rs1) + (uint32_t)imm;

                if (addr & 1)
                    throw runtime_error("Misaligned LHU");

                uint16_t data = mem.read16(addr);
                rf.writeValue(rd, (uint32_t)data);
                trace_line("lhu " +
                           string(regname(rd)) + ", " +
                           to_string(imm) + "(" +
                           string(regname(rs1)) + ")");
                stats.bytes_read += 2;
                stats.loads++;
                stats.instret++;
                return true;
            }
            default:
                break;
            }
            break;
        }

        // R-Type
        case 0x33:
        {
            switch (funct3)
            {
            case 0x0:
            {
                // ADD
                if (funct7 == 0x00)
                {
                    uint32_t v = rf.getValue(rs1) + rf.getValue(rs2);
                    rf.writeValue(rd, v);
                    trace_line("add " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                // SUB
                else if (funct7 == 0x20)
                {
                    uint32_t v = rf.getValue(rs1) - rf.getValue(rs2);
                    rf.writeValue(rd, v);
                    trace_line("sub " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                break;
            }
            // SLL
            case 0x1:
            {
                if (funct7 == 0x00)
                {
                    uint32_t v = rf.getValue(rs1) << (rf.getValue(rs2) & 0x1F);
                    rf.writeValue(rd, v);
                    trace_line("sll " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                break;
            }
            // SLT
            case 0x2:
            {
                if (funct7 == 0x00)
                {
                    int32_t a = (int32_t)rf.getValue(rs1);
                    int32_t b = (int32_t)rf.getValue(rs2);
                    uint32_t v = (a < b) ? 1 : 0;
                    rf.writeValue(rd, v);
                    trace_line("slt " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                break;
            }
            // SLTU
            case 0x3:
            {
                if (funct7 == 0x00)
                {
                    uint32_t v = rf.getValue(rs1) < rf.getValue(rs2) ? 1 : 0;
                    rf.writeValue(rd, v);
                    trace_line("sltu " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                break;
            }
            // XOR
            case 0x4:
            {
                if (funct7 == 0x00)
                {
                    uint32_t v = rf.getValue(rs1) ^ rf.getValue(rs2);
                    rf.writeValue(rd, v);
                    trace_line("xor " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                break;
            }
            case 0x5:
            {
                if (funct7 == 0x00)
                {
                    // SRL
                    uint32_t v = rf.getValue(rs1) >> (rf.getValue(rs2) & 0x1F);
                    rf.writeValue(rd, v);
                    trace_line("srl " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                else if (funct7 == 0x20)
                {
                    // SRA
                    int32_t sv = (int32_t)rf.getValue(rs1);
                    uint32_t v = (uint32_t)(sv >> (rf.getValue(rs2) & 0x1F));
                    rf.writeValue(rd, v);
                    trace_line("sra " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                break;
            }
            // OR
            case 0x6:
            {
                if (funct7 == 0x00)
                {
                    uint32_t v = rf.getValue(rs1) | rf.getValue(rs2);
                    rf.writeValue(rd, v);
                    trace_line("or " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                break;
            }
            // AND
            case 0x7:
            {
                if (funct7 == 0x00)
                {
                    uint32_t v = rf.getValue(rs1) & rf.getValue(rs2);
                    rf.writeValue(rd, v);
                    trace_line("and " +
                               string(regname(rd)) + ", " +
                               string(regname(rs1)) + ", " +
                               string(regname(rs2)));
                    stats.instret++;
                    return true;
                }
                break;
            }
            }
            break;
        }

        // LUI
        case 0x37:
        {
            uint32_t imm = inst & 0xFFFFF000u;
            rf.writeValue(rd, imm);
            trace_line("lui " +
                       string(regname(rd)) + ", " +
                       hex32((imm)));
            stats.instret++;
            return true;
        }

        // AUIPC
        case 0x17:
        {
            uint32_t imm = inst & 0xFFFFF000u;
            rf.writeValue(rd, old_pc + imm);
            trace_line("auipc " +
                       string(regname(rd)) + ", " +
                       hex32((imm)));
            stats.instret++;
            return true;
        }

        // S-Type
        case 0x23:
        { // STORE
            int32_t imm = imm_s(inst);
            uint32_t addr = rf.getValue(rs1) + (uint32_t)imm;
            uint32_t data = rf.getValue(rs2);

            switch (funct3)
            {
            case 0x0: // SB
                mem.write8(addr, (uint8_t)data);
                trace_line("sb " + string(regname(rs2)) + ", " +
                           to_string(imm) + "(" + string(regname(rs1)) + ")");
                stats.bytes_written += 1;
                stats.stores++;
                stats.instret++;
                return true;

            case 0x1: // SH
                if (addr & 1)
                    throw runtime_error("Misaligned SH");
                mem.write16(addr, (uint16_t)data);
                trace_line("sh " + string(regname(rs2)) + ", " +
                           to_string(imm) + "(" + string(regname(rs1)) + ")");
                stats.bytes_written += 2;
                stats.stores++;
                stats.instret++;
                return true;

            case 0x2: // SW
                if (addr & 3)
                    throw runtime_error("Misaligned SW");
                mem.write32(addr, data);
                trace_line("sw " + string(regname(rs2)) + ", " +
                           to_string(imm) + "(" + string(regname(rs1)) + ")");
                stats.bytes_written += 4;
                stats.stores++;
                stats.instret++;
                return true;

            default:
                break;
            }
            break;
        }

        // B-Type
        case 0x63:
        {
            int32_t off = imm_b(inst);
            switch (funct3)
            {
            // BEQ
            case 0x0: // BEQ
            {
                if (rf.getValue(rs1) == rf.getValue(rs2))
                {
                    pc = old_pc + (uint32_t)off;
                    stats.taken_branches++;
                }

                trace_line("beq " +
                           string(regname(rs1)) + ", " +
                           string(regname(rs2)) + ", " +
                           to_string(off));
                stats.branches++;
                stats.instret++;
                return true;
            }
            // BNE
            case 0x1:
            {
                if (rf.getValue(rs1) != rf.getValue(rs2))
                {
                    pc = old_pc + (uint32_t)off;
                    stats.taken_branches++;
                }

                trace_line("bne " +
                           string(regname(rs1)) + ", " +
                           string(regname(rs2)) + ", " +
                           to_string(off));
                stats.branches++;
                stats.instret++;
                return true;
            }
            // BLT
            case 0x4:
            {
                if ((int32_t)rf.getValue(rs1) < (int32_t)rf.getValue(rs2))
                {
                    pc = old_pc + (uint32_t)off;
                    stats.taken_branches++;
                }

                trace_line("blt " +
                           string(regname(rs1)) + ", " +
                           string(regname(rs2)) + ", " +
                           to_string(off));
                stats.branches++;
                stats.instret++;
                return true;
            }
            // BGE
            case 0x5:
            {
                if ((int32_t)rf.getValue(rs1) >= (int32_t)rf.getValue(rs2))
                {
                    pc = old_pc + (uint32_t)off;
                    stats.taken_branches++;
                }

                trace_line("bge " +
                           string(regname(rs1)) + ", " +
                           string(regname(rs2)) + ", " +
                           to_string(off));
                stats.branches++;
                stats.instret++;
                return true;
            }
            // BLTU
            case 0x6:
            {
                if (rf.getValue(rs1) < rf.getValue(rs2))
                {
                    pc = old_pc + (uint32_t)off;
                    stats.taken_branches++;
                }

                trace_line("bltu " +
                           string(regname(rs1)) + ", " +
                           string(regname(rs2)) + ", " +
                           to_string(off));
                stats.branches++;
                stats.instret++;
                return true;
            }
            // BGEU
            case 0x7:
            {
                if (rf.getValue(rs1) >= rf.getValue(rs2))
                {
                    pc = old_pc + (uint32_t)off;
                    stats.taken_branches++;
                }

                trace_line("bgeu " +
                           string(regname(rs1)) + ", " +
                           string(regname(rs2)) + ", " +
                           to_string(off));
                stats.branches++;
                stats.instret++;
                return true;
            }
            default:
                break;
            }
            break;
        }

        // JALR
        case 0x67:
        {
            if (funct3 == 0x0)
            {
                int32_t imm = imm_i(inst);
                uint32_t target = (rf.getValue(rs1) + (uint32_t)imm) & ~1u;

                rf.writeValue(rd, pc); // pc already = old_pc + 4
                pc = target;

                trace_line("jalr " + string(regname(rd)) + ", " +
                           to_string(imm) + "(" + string(regname(rs1)) + ")");
                stats.jumps++;
                stats.instret++;
                return true;
            }
            break;
        }

        // Jal
        case 0x6F:
        {
            int32_t off = imm_j(inst);
            rf.writeValue(rd, pc);       // rd gets return addr (pc already pc+4)
            pc = old_pc + (uint32_t)off; // jump target relative to old_pc
            trace_line("jal " +
                       string(regname(rd)) +
                       ", " + to_string(off));
            stats.jumps++;
            stats.instret++;
            return true;
        }
        }

        // If unsupported, stop (for now)
        std::cerr << "Illegal/unsupported instruction @ PC=0x"
                  << std::hex << old_pc << " inst=0x" << inst << "\n";
        return false;
    }
};

/************************************************* ELF LOADER *************************************************/
struct Range
{
    uint32_t lo, hi;
}; // [lo, hi)
struct ElfInfo
{
    uint32_t entry;
    vector<Range> load_ranges;
    vector<Range> exec_ranges;
};

static inline uint16_t rd16le(const uint8_t *p)
{
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
static inline uint32_t rd32le(const uint8_t *p)
{
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

ElfInfo loadElf32Riscv(const string &path, Memory &memory)
{
    // Read whole file
    ifstream f(path, ios::binary);
    if (!f)
        throw runtime_error("Failed to open ELF: " + path);

    vector<uint8_t> bytes((istreambuf_iterator<char>(f)),
                          istreambuf_iterator<char>());

    auto need = [&](size_t off, size_t n)
    {
        if (off + n > bytes.size())
            throw runtime_error("ELF truncated/corrupt");
    };

    need(0, 16);
    if (!(bytes[0] == 0x7f && bytes[1] == 'E' && bytes[2] == 'L' && bytes[3] == 'F'))
        throw runtime_error("Not an ELF file");

    const uint8_t ei_class = bytes[4]; // 1 = ELF32
    const uint8_t ei_data = bytes[5];  // 1 = little-endian
    if (ei_class != 1)
        throw runtime_error("Not ELF32");
    if (ei_data != 1)
        throw runtime_error("Not little-endian ELF");

    // ELF32 header offsets
    const uint16_t e_type = rd16le(&bytes[0x10]);
    const uint16_t e_machine = rd16le(&bytes[0x12]);
    const uint32_t e_version = rd32le(&bytes[0x14]);
    const uint32_t e_entry = rd32le(&bytes[0x18]);
    const uint32_t e_phoff = rd32le(&bytes[0x1C]);
    const uint16_t e_phentsz = rd16le(&bytes[0x2A]);
    const uint16_t e_phnum = rd16le(&bytes[0x2C]);

    // Basic validation
    if (e_type != 2)
        throw runtime_error("ELF is not EXEC (ET_EXEC)");
    if (e_machine != 243)
        throw runtime_error("ELF is not RISC-V (EM_RISCV)");
    if (e_version != 1)
        throw runtime_error("Unexpected ELF version");

    ElfInfo info{};
    info.entry = e_entry;

    constexpr uint32_t PT_LOAD = 1;
    constexpr uint32_t PF_X = 0x1; // executable segment flag

    // Program headers
    for (uint16_t i = 0; i < e_phnum; i++)
    {
        const uint32_t ph = e_phoff + uint32_t(i) * uint32_t(e_phentsz);
        need(ph, 32);

        const uint32_t p_type = rd32le(&bytes[ph + 0x00]);
        const uint32_t p_offset = rd32le(&bytes[ph + 0x04]);
        const uint32_t p_vaddr = rd32le(&bytes[ph + 0x08]);
        const uint32_t p_filesz = rd32le(&bytes[ph + 0x10]);
        const uint32_t p_memsz = rd32le(&bytes[ph + 0x14]);
        const uint32_t p_flags = rd32le(&bytes[ph + 0x18]);

        if (p_type != PT_LOAD)
            continue;

        need(p_offset, p_filesz);

        // Copy file bytes into simulated memory
        for (uint32_t j = 0; j < p_filesz; j++)
        {
            memory.write8(p_vaddr + j, bytes[p_offset + j]);
        }
        // Zero-fill extra (BSS) bytes
        for (uint32_t j = p_filesz; j < p_memsz; j++)
        {
            memory.write8(p_vaddr + j, 0);
        }

        // Record ranges
        info.load_ranges.push_back({p_vaddr, p_vaddr + p_memsz});
        if (p_flags & PF_X)
            info.exec_ranges.push_back({p_vaddr, p_vaddr + p_memsz});
    }

    return info;
}

/***************************** Other Helper Functions *****************************/
static void dump_step_writes(const Memory &mem)
{
    for (const auto &w : mem.get_step_writes())
    {
        std::cout << "\033[1;34m  MEM: [" << hex32(w.addr) << "] <= " << hex32(w.value)
                  << " (" << (w.size == 1 ? "sb" : w.size == 2 ? "sh"
                                                               : "sw")
                  << ")\033[1;0m\n";
    }
}

static void dump_used_words(const Memory &mem)
{
    std::unordered_set<uint32_t> used_words;
    for (uint32_t b : mem.get_used_bytes())
        used_words.insert(b & ~3u);

    std::vector<uint32_t> addrs(used_words.begin(), used_words.end());
    std::sort(addrs.begin(), addrs.end());

    uint32_t prev = 0;
    bool first = true;

    for (uint32_t a : addrs)
    {
        if (!first && a != prev + 4)
            std::cout << "\n"; // region separator

        std::cout << hex32(a) << ": " << hex32(mem.read32(a)) << " = " << mem.read32(a) << "\n";

        prev = a;
        first = false;
    }
}

static void dump_all_range_words(const Memory &mem, uint32_t base, uint32_t end)
{
    // align to words
    uint32_t a = base & ~3u;
    uint32_t e = (end + 3u) & ~3u;

    for (; a < e; a += 4)
    {
        std::cout << hex32(a) << ": " << hex32(mem.read32(a)) << " = " << mem.read32(a) << "\n";
    }
}

static void dump_rf_step_used(const RegisterFile &rf)
{
    const auto &w = rf.get_step_writes();
    for (const auto &p : w)
    {
        int r = p.first;
        uint32_t v = p.second;
        std::cout << "  RF: " << regname(r) << " = " << hex32(v) << "\n";
    }
}

static void dump_rf_used_end(const RegisterFile &rf)
{
    std::vector<int> regs(rf.get_used_regs().begin(), rf.get_used_regs().end());
    std::sort(regs.begin(), regs.end());

    for (int r : regs)
        std::cout << regname(r) << " = " << hex32(rf.getValue(r)) << "\n";
    cout << "\n";
}

static void dump_rf_all(const RegisterFile &rf)
{
    for (int r = 0; r < 32; r++)
    {
        std::cout << regname(r) << "=" << hex32(rf.getValue(r)) << ((r % 8 == 7) ? "\n" : "\t");
    }
    std::cout << "\n";
}

/************************************************************************************/

int main(int argc, char **argv)
{
    Config cfg;
    try
    {
        cfg = parse_args(argc, argv);
    }
    catch (const exception &e)
    {
        cerr << "Error: " << e.what() << "\n";
        print_help();
        return 1;
    }

    Memory mem;
    uint32_t entry = 0;

    ElfInfo elf = loadElf32Riscv(cfg.elf_path, mem);
    CPU cpu;
    cpu.pc = elf.entry;

    // find the exec range containing entry
    Range code{};
    bool found = false;
    for (auto r : elf.exec_ranges)
    {
        if (cpu.pc >= r.lo && cpu.pc < r.hi)
        {
            code = r;
            found = true;
            break;
        }
    }
    if (!found)
    {
        cerr << "Entry not in executable range\n";
        return 1;
    }

    // Simply dump all instructions
    if (cfg.dump_instrs == true)
    {
        cout << "Reading Instructions:" << endl;

        for (uint64_t steps = 0; steps < cfg.max_steps && cpu.pc + 3 < code.hi; steps++)
        {
            uint32_t inst = mem.read32(cpu.pc);
            cout << hex << cpu.pc << ": " << hex << setw(8) << setfill('0') << inst << "\n";
            cpu.pc += 4;
        }
    }
    else
    {
        for (uint64_t steps = 0; steps < cfg.max_steps; steps++)
        {
            // stop if PC leaves exec range
            if (!(cpu.pc >= code.lo && cpu.pc + 3 < code.hi))
            {
                cerr << "PC left executable range: 0x" << hex << cpu.pc << "\n";
                break;
            }

            mem.begin_step(); // NEW: clear per-step write log
            if (!cpu.step(mem, cfg))
                break;

            /*** When tracing dump requested memory ***/
            if (cfg.trace)
            {
                if (cfg.show_memory == DumpMode::USED || cfg.show_memory == DumpMode::ALL)
                {
                    dump_step_writes(mem);
                }
                if (cfg.show_rf == DumpMode::USED) // per your semantics: USED => per-step writes when tracing
                {
                    dump_rf_step_used(cpu.rf);
                }
                else if (cfg.show_rf == DumpMode::ALL)
                {
                    dump_rf_all(cpu.rf);
                }
            }
        }

        /*** Final dumping rules ***/
        if (cfg.show_memory == DumpMode::USED)
        {
            cout << "\033[1;36mMEM(\033[1;34mUsed\033[1;36m):\n";
            dump_used_words(mem);
        }
        else if (cfg.show_memory == DumpMode::ALL)
        {
            cout << "\033[1;36mMEM(\033[1;34mAll\033[1;36m):\n";
            dump_all_range_words(mem, cfg.mem_base, cfg.mem_end);
        }
        cout << "\033[1;35m\n";
        if (cfg.show_rf == DumpMode::USED)
        {
            std::cout << "RF(used):\n";
            dump_rf_used_end(cpu.rf);
        }
        else if (cfg.show_rf == DumpMode::ALL)
        {
            std::cout << "RF(all):\n";
            dump_rf_all(cpu.rf);
        }
        cout << "\033[1;0m";
    }

    /*** Show statistics at the end of simulation ***/
    if (cfg.show_stats)
    {
        cout << "\033[1;33m\n";
        cout << dec;
        cout << "--- Simulation Summary ---\n";
        cout << "instret:        " << cpu.stats.instret << "\n";
        cout << "loads:          " << cpu.stats.loads << "\n";
        cout << "stores:         " << cpu.stats.stores << "\n";
        cout << "branches:       " << cpu.stats.branches << "\n";
        cout << "taken_branches: " << cpu.stats.taken_branches << "\n";
        cout << "jumps:          " << cpu.stats.jumps << "\n";
        cout << "bytes_read:     " << cpu.stats.bytes_read << "\n";
        cout << "bytes_written:  " << cpu.stats.bytes_written << "\n";
        cout << "\033[1;0m\n";
    }

    return 0;
}
