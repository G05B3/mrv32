#include <cstdint>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <iomanip>
#include <string>

#include <iostream>

using namespace std;

enum class DumpMode
{
    NONE,
    ACTIVE,
    ALL
};

static void print_help() {
    std::cout <<
    "MRV32 Instruction Set Simulator\n"
    "Usage:\n"
    "  mrv_iss [options] <program.elf>\n\n"
    "Options:\n"
    "  -h, --help\n"
    "      Show this help message and exit\n\n"
    "  --max-steps=<n>\n"
    "      Maximum number of instructions to execute (default: 1000)\n\n"
    "  --trace=(true|false)\n"
    "      Enable per-instruction trace output (default: false)\n\n"
    "  --show-memory=(none|active|all)\n"
    "      Control memory dump behavior\n"
    "        none   : do not display memory\n"
    "        active : display memory written in the current step\n"
    "        all    : display full memory range each step\n\n"
    "  --mem-addr-base=<addr>\n"
    "      Base address for --show-memory=all (default: 0x00000000)\n\n"
    "  --mem-addr-end=<addr>\n"
    "      End address for --show-memory=all (default: 0x000000FF)\n\n"
    "  --show-register-file=(none|active|all)\n"
    "      Control register file dump behavior\n\n"
    "Examples:\n"
    "  mrv_iss test.elf\n"
    "  mrv_iss --trace=true --max-steps=10000 test.elf\n"
    "  mrv_iss --trace=true --show-memory=active test.elf\n";
}


struct Config
{
    string elf_path;

    uint64_t max_steps = 1000; // default
    bool trace = false;

    DumpMode show_memory = DumpMode::NONE;
    DumpMode show_rf = DumpMode::NONE;

    uint32_t mem_base = 0x00000000; // default range for ALL
    uint32_t mem_end = 0x000000FF;
};

static bool starts_with(const string &s, const string &p)
{
    return s.rfind(p, 0) == 0;
}

static DumpMode parse_dump_mode(const string &s)
{
    if (s == "none")
        return DumpMode::NONE;
    if (s == "active")
        return DumpMode::ACTIVE;
    if (s == "all")
        return DumpMode::ALL;
    throw runtime_error("Invalid dump mode: " + s);
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
            cfg.show_rf = parse_dump_mode(a.substr(22));
        }
        else if (starts_with(a, "--mem-addr-base="))
        {
            cfg.mem_base = stoul(a.substr(16), nullptr, 0);
        }
        else if (starts_with(a, "--mem-addr-end="))
        {
            cfg.mem_end = stoul(a.substr(15), nullptr, 0);
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

/************************************************* REGISTER FILE *************************************************/

class RegisterFile
{
public:
    RegisterFile(int size);
    ~RegisterFile();

    int getSize() const;
    uint32_t getValue(int index);
    void writeValue(int index, int value);

private:
    uint32_t *rf;
    int sz;
};

RegisterFile::RegisterFile(int size)
{
    // Constructor implementation
    sz = size;
    rf = new uint32_t[size];
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

void RegisterFile::writeValue(int index, int value)
{
    if (index < 0 || index >= sz)
    {
        cerr << "Index out of bounds" << endl;
        return; // or handle error appropriately
    }
    rf[index] = value;
}

uint32_t RegisterFile::getValue(int index)
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
    void write8(uint32_t addr, uint8_t val) { mem[addr] = val; }

    uint8_t read8(uint32_t addr) const
    {
        auto it = mem.find(addr);
        return (it == mem.end()) ? 0 : it->second;
    }

    uint32_t read32(uint32_t addr) const
    {
        // little-endian
        uint32_t b0 = read8(addr);
        uint32_t b1 = read8(addr + 1);
        uint32_t b2 = read8(addr + 2);
        uint32_t b3 = read8(addr + 3);
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
    }

private:
    unordered_map<uint32_t, uint8_t> mem;
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
    uint32_t pc = elf.entry;

    // find the exec range containing entry
    Range code{};
    bool found = false;
    for (auto r : elf.exec_ranges)
    {
        if (pc >= r.lo && pc < r.hi)
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

    for (uint64_t steps = 0; steps < cfg.max_steps && pc + 3 < code.hi; steps++)
    {
        uint32_t inst = mem.read32(pc);
        cout << hex << pc << ": " << hex << setw(8) << setfill('0') << inst << "\n";
        pc += 4;
    }

    return 0;
}
