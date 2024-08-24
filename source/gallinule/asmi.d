module gallinule.asmi;

import std.string : splitLines, strip, split, replace;
import tern.string : startsWith, endsWith, toUpper;
import tern.algorithm.mutation : filter;
import std.ascii;
import std.conv;
    
pure string parse(string path)()
{
    string ret;
    foreach (line; import(path).splitLines)
    {
        if (line.strip.length == 0)
            continue;

        string[] parameters;
        string generics;
        string conditional;
        string operands;
        string mnemonic;
        string[] mappings = ["0", null, "128", "1", "0"];
        string[] details;
        string emit;
        string combo;

        foreach (i, word; line.split(' '))
        {
            if (word.startsWith("0x"))
            {
                if (word.endsWith('p'))
                    mappings[4] = word[0..$-1];
                else
                    emit ~= word~", ";
            }
            else if (word.startsWith('$'))
            {
                string operand = 'o'~word[1..$];
                emit ~= operand~", ";
                parameters ~= operand;
            }
            else if (word.startsWith('['))
                mappings[3] = 'M'~word[1..$-1].toUpper;
            else if (word.startsWith("vex"))
            {
                mappings[2] = word[3..6];
                mappings[1] ~= "VEX ^ "~(word.endsWith(".i") ? "INT ^ " : null);
            }
            else if (word.startsWith("mvex"))
            {
                mappings[2] = word[3..6];
                mappings[1] ~= "MVEX ^ "~(word.endsWith(".i") ? "INT ^ " : null);
            }
            else if (word.startsWith("evex"))
            {
                mappings[2] = word[3..6];
                mappings[1] ~= "EVEX ^ "~(word.endsWith(".i") ? "INT ^ " : null);
            }
            else if (word.endsWith("/e"))
                mappings[1] ~= "ENCODED ^ ";
            else if (word.endsWith("/f"))
                mappings[1] ~= "FLIP ^ ";
            else if (word.endsWith("/rx"))
                mappings[1] ~= "NO_REX ^ ";
            else if (word.startsWith('/'))
                mappings[0] = word[1..$];
            else if ((word.startsWith('r') && (word.endsWith("xx") || word[1..$].filter!(x => x.isDigit).length == word.length - 1)) ||
                (word.startsWith('i') && (word.endsWith("xx") || word[1..$].filter!(x => x.isDigit).length == word.length - 1)) ||
                (word.startsWith('m') && (word.endsWith("xx") || word[1..$].filter!(x => x.isDigit).length == word.length - 1)) ||
                (word.startsWith("rm") && (word.endsWith("xx") || word[2..$].filter!(x => x.isDigit).length == word.length - 2)) || 
                word == "st" || word == "dr" || word == "cr")
            {
                if (mnemonic == null && i > 0)
                {
                    mnemonic = line.split(' ')[i - 1];

                    if (emit.length > 0)
                        emit = emit[0..$-mnemonic.length-2];
                    else if (combo.length > 0)
                        combo = combo[0..$-mnemonic.length];
                }
                else if (i == 0)
                    throw new Throwable("Mnemonic must come before parameters!");
                    
                details ~= word;
            }
            else if (emit.length > 0)
                emit ~= word~", ";
            else
                combo ~= word;
        }

        foreach (i, detail; details)
        {
            string type = detail.toUpper;
            if (type.startsWith("RM"))
            {
                type = type~i.to!string;
                generics ~= type~", ";

                if (type == "RMXX")
                    conditional ~= "(isInstanceOf!(Register, "~type~") | isInstanceOf!(Memory, "~type~"))";
                else
                    conditional ~= "valid!("~type~", "~detail[2..$]~')';
            }
            else if (type == "IXX")
            {
                type = type~i.to!string;
                generics ~= type~", ";
                conditional ~= "isIntegral!"~type;
            }
            else if (type == "RXX")
            {
                type = type~i.to!string;
                generics ~= type~", ";
                conditional ~= "isInstanceOf!(Register, "~type~')';
            }
            else if (type == "MXX")
            {
                type = type~i.to!string;
                generics ~= type~", ";
                conditional ~= "isInstanceOf!(Memory, "~type~')';
            }

            switch (type)
            {
                case "I8":
                    type = "ubyte";
                    break;

                case "I16":
                    type = "ushort";
                    break;

                case "I32":
                    type = "uint";
                    break;

                case "I64":
                    type = "ulong";
                    break;

                default:
                    if (type.startsWith('I'))
                        throw new Throwable("Invalid integral parameter '"~detail~"'!");
                    break;
            }
            operands ~= type~' '~parameters.filter!(x => x.endsWith(i.to!string))[0]~", ";
        }

        if (generics.length > 0)
        {
            generics = '('~generics[0..$-2]~')';
            conditional = "if ("~conditional~") ";
        }

        if (mappings[1] == null)
            mappings[1] = "0";
        else
            mappings[1] = mappings[1][0..$-3];

        if (operands.length > 0)
            operands = operands[0..$-2];

        
        if (emit.length > 0)
        {
            if (mnemonic == null)
            {
                mnemonic = line.split(' ')[$-1];
                emit = emit[0..$-mnemonic.length-2];
            }

            string mapping = mappings[0]~", "~mappings[1]~", "~mappings[2]~", "~mappings[3]~", "~mappings[4];
            emit = "emit!("~mapping~")("~emit[0..$-2]~')';
        }
        else if (combo.length > 0)
        {
            if (mnemonic == null)
            {
                mnemonic = line.split(' ')[$-1];
                combo = combo[0..$-mnemonic.length];
            }

            emit = combo.replace(")", ") + ")[0..$-3];
        }
        else
            throw new Throwable("No instruction emission or combo data found!");

        string attr = details.length == 0 ? null : "@("~details.to!string[1..$-1]~")\n";
        string _body = "auto "~mnemonic~generics~"("~operands~") "~conditional~"=> "~emit~";\n\n";
        ret ~= attr~_body;
    }
    return ret[0..$-2];
}