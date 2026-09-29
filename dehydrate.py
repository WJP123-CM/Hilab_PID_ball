import sys
import re

def extract_registers(text):
    regs = re.findall(r'(PC|LR|CFSR|HFSR|BFAR|MSP|PSP)\s*[:=]\s*(0x[0-9A-Fa-f]+)', text)
    return '; '.join([f'{r[0]}={r[1]}' for r in regs]) if regs else None

def truncate_array(text):
    pattern = r'(\w+\s+\w+\[\s*\d+\s*\]\s*=\s*\{)([^}]*)(\})'
    def repl(m):
        body = [x.strip() for x in m.group(2).split(',') if x.strip()]
        if len(body) <= 8:
            return m.group(0)
        head = ','.join(body[:4])
        tail = ','.join(body[-4:])
        return f"{m.group(1)}{head}, /* ... omitted {len(body)-8} elements ... */,{tail}{m.group(3)}"
    return re.sub(pattern, repl, text, flags=re.DOTALL)

def clean(text):
    text = re.sub(r'//.*?$', '', text, flags=re.M)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'^\s*$\n', '', text, flags=re.M)
    text = re.sub(r'[ \t]+', ' ', text)
    return text

def main():
    print("粘贴代码，完成后单独输入 END 并回车：")
    lines = []
    while True:
        try:
            line = input()
        except EOFError:
            break
        if line.strip() == "END":
            break
        lines.append(line)

    raw = "\n".join(lines)
    if not raw.strip():
        input("未检测到输入（可能你只输入了END），按回车退出...")
        return

    # 寄存器提取
    regs = extract_registers(raw)
    if regs:
        lines_split = raw.splitlines()
        if len(lines_split) > 60:
            core = '\n'.join(lines_split[:30] + ['... (truncated) ...'] + lines_split[-30:])
        else:
            core = raw
        output = f"[Registers: {regs}]\n{clean(core)}"
    else:
        text = truncate_array(raw)
        lines_split = text.splitlines()
        if len(lines_split) > 500:
            text = '\n'.join(lines_split[:30] + ['... (truncated) ...'] + lines_split[-20:])
        output = clean(text)

    print("\n--- 脱水结果（直接复制） ---")
    print(output)
    input("按回车键退出...")

if __name__ == "__main__":
    main()
