#pragma once

inline std::string opcode_to_string(noentiendo::opcode type)
{
#define STRINGIFY(x)             \
    case noentiendo::opcode::x: \
        return #x;

    switch (type)
    {
        STRINGIFY(ADC);
        STRINGIFY(AND);
        STRINGIFY(ASL);
        STRINGIFY(BCC);
        STRINGIFY(BCS);
        STRINGIFY(BEQ);
        STRINGIFY(BIT);
        STRINGIFY(BMI);    
        STRINGIFY(BNE);
        STRINGIFY(BPL);
        STRINGIFY(BRK);
        STRINGIFY(BVC);
        STRINGIFY(BVS);
        STRINGIFY(CLC);
        STRINGIFY(CLD);
        STRINGIFY(CLI);
        STRINGIFY(CLV);
        STRINGIFY(CMP);
        STRINGIFY(CPX);
        STRINGIFY(CPY);
        STRINGIFY(DEC);
        STRINGIFY(DEX);
        STRINGIFY(DEY);
        STRINGIFY(EOR);
        STRINGIFY(INC);
        STRINGIFY(INX);
        STRINGIFY(INY);
        STRINGIFY(JMP);
        STRINGIFY(JSR);
        STRINGIFY(LDA);
        STRINGIFY(LDX);
        STRINGIFY(LDY);
        STRINGIFY(LSR);
        STRINGIFY(NOP);
        STRINGIFY(ORA);
        STRINGIFY(PHA);
        STRINGIFY(PHP);
        STRINGIFY(PLA);
        STRINGIFY(PLP);
        STRINGIFY(ROL);
        STRINGIFY(ROR);
        STRINGIFY(RTI);
        STRINGIFY(RTS);
        STRINGIFY(SBC);
        STRINGIFY(SEC);
        STRINGIFY(SED);
        STRINGIFY(SEI);
        STRINGIFY(STA);
        STRINGIFY(STX);
        STRINGIFY(STY);
        STRINGIFY(TAX);
        STRINGIFY(TAY);
        STRINGIFY(TSX);
        STRINGIFY(TXA);
        STRINGIFY(TXS);
        STRINGIFY(TYA);
        case noentiendo::opcode::None: 
        default:
            return "XXX";
    }

#undef stringify
}
