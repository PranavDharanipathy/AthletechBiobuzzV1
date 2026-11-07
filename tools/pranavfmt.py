#!/usr/bin/env python3
"""Reformat Java to Pranav's AthletechPremiers2526 style (see DEV_NOTES code style).

- a statement wrapped across lines (hanging args, trailing/leading operator) gets joined onto one line
- kept: block lists (line ends with "(", items on their own lines, ")" starts its own line),
  builder chains (one .call() per line), array initializer lists, anonymous class / lambda bodies, block comments
- `} else` / `} else if` -> `}` then `else` on the next line
- methods/constructors: blank line after `{` when the body has 2+ lines, none for one-liners
Usage: python tools/pranavfmt.py [--check] files...
Only changes formatting: compare `javap -c -p` before/after if in doubt.
"""
import re
import sys

BINARY_END = ('&&', '||', '+', '-', '*', '/', '?', '=', '==', '!=', '<', '>', '<=', '>=', '%', '&', '|', '^')
LEADING = ('&&', '||', '?', '+', ':', '-', '*', '/')  #a leading . is a builder chain, kept one call per line


def split_code_comment(line, in_block):
    """returns (code, comment, in_block_after, has_block_comment). code excludes a trailing // comment."""
    i, n = 0, len(line)
    code = []
    has_block = in_block
    while i < n:
        if in_block:
            j = line.find('*/', i)
            if j < 0:
                return ''.join(code), '', True, True
            code.append(line[i:j + 2])
            i = j + 2
            in_block = False
            continue
        c = line[i]
        if c == '"' or c == "'":
            q = c
            j = i + 1
            while j < n:
                if line[j] == '\\':
                    j += 2
                    continue
                if line[j] == q:
                    break
                j += 1
            code.append(line[i:j + 1])
            i = j + 1
            continue
        if line.startswith('//', i):
            return ''.join(code), line[i:], False, has_block
        if line.startswith('/*', i):
            has_block = True
            in_block = True
            code.append('/*')
            i += 2
            continue
        code.append(c)
        i += 1
    return ''.join(code), '', in_block, has_block


def strip_strings(code):
    return re.sub(r'"(\\.|[^"\\])*"|\'(\\.|[^\'\\])*\'', '""', code)


def bracket_delta(code, stack):
    s = strip_strings(code)
    s = re.sub(r'/\*.*?\*/', '', s)
    for ch in s:
        if ch in '([{':
            stack.append(ch)
        elif ch in ')]}':
            if stack:
                stack.pop()
    return stack


def ends_with_operator(code):
    c = code.rstrip()
    if c.endswith('->'):
        return False
    if c.endswith('++') or c.endswith('--'):
        return False
    for op in sorted(BINARY_END, key=len, reverse=True):
        if c.endswith(op):
            if op == '>' and re.search(r'<[\w\s,.<>?\[\]]*>$', c):
                return False  #generic
            if op in ('*', '/') and c.endswith('*/'):
                return False
            return True
    return False


def starts_with_leading(code):
    c = code.lstrip()
    if c.startswith('::') or c.startswith('//') or c.startswith('/*') or c.startswith('*'):
        return False
    if c.startswith('++') or c.startswith('--'):
        return False
    return any(c.startswith(op) for op in LEADING)


def join_pieces(a, b):
    a = a.rstrip()
    b = b.strip()
    if a.endswith('(') or a.endswith('[') or b.startswith(')') or b.startswith(']') or b.startswith('.') or b.startswith(',') or b.startswith(';'):
        return a + b
    return a + ' ' + b


def join_wrapped(lines, warnings, path):
    out = []
    i = 0
    in_block = False
    stack = []
    while i < len(lines):
        line = lines[i]
        code, comment, in_block_after, has_block = split_code_comment(line, in_block)
        if has_block or in_block:
            in_block = in_block_after
            bracket_delta(code, stack)
            out.append(line)
            i += 1
            continue
        in_block = in_block_after

        stripped = code.rstrip()

        #Pranav's block form: opener ends with "(", one item per line, closer line starts with ")" -> keep it
        if stripped.endswith('('):
            k = find_block_end(lines, i)
            if k is not None:
                for t in range(i, k + 1):
                    out.append(lines[t])
                    bracket_delta(split_code_comment(lines[t], False)[0], stack)
                i = k + 1
                continue

        before = list(stack)
        bracket_delta(code, stack)

        def continues(code_now, stack_now, nxt_code):
            s = code_now.rstrip()
            if not s:
                return False
            if s.endswith('{') or s.endswith(';'):
                return False
            if s.lstrip().startswith('@') and not stack_now:
                return False
            if stack_now and stack_now[-1] in '([':
                return True
            if s.endswith('}'):
                return False
            if stack_now and stack_now[-1] == '{' and s.endswith(','):
                return False
            if ends_with_operator(s):
                return True
            if nxt_code is not None and starts_with_leading(nxt_code):
                return True
            return False

        nxt = lines[i + 1] if i + 1 < len(lines) else None
        nxt_code = split_code_comment(nxt, False)[0] if nxt is not None else None
        if not continues(stripped, stack, nxt_code):
            out.append(line)
            i += 1
            continue

        #gather the statement
        indent = re.match(r'\s*', line).group(0)
        joined = stripped.strip()
        comments = [comment.strip()] if comment.strip() else []
        j = i + 1
        ok = True
        cur_stack = list(stack)
        while j < len(lines):
            nl = lines[j]
            ncode, ncomment, nblock_after, nhas_block = split_code_comment(nl, False)
            if nhas_block or nl.strip() == '':
                ok = False
                break
            if ncode.rstrip().endswith('{') and not re.search(r'\)\s*\{$', ncode.rstrip()) and 'new ' in ncode and '(' in ncode:
                ok = False  #anonymous class starts inside the statement
                break
            joined = join_pieces(joined, ncode)
            if ncomment.strip():
                comments.append(ncomment.strip())
            bracket_delta(ncode, cur_stack)
            j += 1
            nn = lines[j] if j < len(lines) else None
            nn_code = split_code_comment(nn, False)[0] if nn is not None else None
            if not continues(ncode.rstrip(), cur_stack, nn_code):
                break
        if not ok or j - i < 2:
            warnings.append('%s:%d: left wrapped (comment block or blank inside)' % (path, i + 1))
            out.append(line)
            stack = before
            bracket_delta(code, stack)
            i += 1
            continue
        if joined.endswith('{') and re.search(r'\bnew\b[^;]*\{$', joined) and not re.search(r'\)\s*\{$', joined):
            pass
        text = indent + joined
        if comments:
            text += ' ' + ' '.join(comments)
        out.append(text)
        stack = cur_stack
        i = j
    return out


def find_block_end(lines, i):
    start = 0
    for ch in strip_strings(split_code_comment(lines[i], False)[0]):
        if ch == '(':
            start += 1
        elif ch == ')':
            start -= 1
    if start <= 0:
        return None
    depth = start
    for t in range(i + 1, min(len(lines), i + 200)):
        code = split_code_comment(lines[t], False)[0]
        for ch in strip_strings(code):
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth < start:
                    return t if code.lstrip().startswith(')') else None
    return None


def one_item_per_line(lines, i, k):
    depth = 1
    for t in range(i + 1, k):
        code = split_code_comment(lines[t], False)[0].rstrip()
        s = strip_strings(code)
        commas = 0
        for idx, ch in enumerate(s):
            if ch in '([{':
                depth += 1
            elif ch in ')]}':
                depth -= 1
            elif ch == ',' and depth == 1 and idx != len(s) - 1:
                commas += 1
        if commas:
            return False
    return True


def split_else(lines):
    out = []
    for line in lines:
        m = re.match(r'^(\s*)\}\s+(else\b.*)$', line)
        if m and not line.strip().startswith('//'):
            out.append(m.group(1) + '}')
            out.append(m.group(1) + m.group(2))
        else:
            out.append(line)
    return out


METHOD = re.compile(r'^\s*(?:@\w+\s+)*(?:(?:public|private|protected|static|final|synchronized|abstract|native)\s+)*(?:<[^>]+>\s+)?[\w<>\[\],.? ]+\s+\w+\s*\([^;]*\)\s*(?:throws\s+[\w., ]+)?\s*\{\s*(//.*)?$')
CTOR = re.compile(r'^\s*(?:(?:public|private|protected)\s+)?[A-Z]\w*\s*\([^;]*\)\s*(?:throws\s+[\w., ]+)?\s*\{\s*(//.*)?$')
CONTROL = re.compile(r'^\s*(if|for|while|switch|catch|synchronized|else|try|do|return|new|throw)\b')


def method_blank_lines(lines):
    out = list(lines)
    i = 0
    while i < len(out):
        line = out[i]
        code = split_code_comment(line, False)[0]
        if (METHOD.match(code) or CTOR.match(code)) and not CONTROL.match(code) and ' new ' not in code and '=' not in code.split('(')[0]:
            #find the body end
            depth = 0
            stack = []
            bracket_delta(code, stack)
            depth = len(stack)
            k = i + 1
            while k < len(out) and depth > 0:
                c = split_code_comment(out[k], False)[0]
                st = []
                for ch in strip_strings(c):
                    if ch == '{':
                        depth += 1
                    elif ch == '}':
                        depth -= 1
                        if depth == 0:
                            break
                if depth == 0:
                    break
                k += 1
            body = [l for l in out[i + 1:k] if l.strip() != '']
            if k >= len(out):
                i += 1
                continue
            first_blank = i + 1 < k and out[i + 1].strip() == ''
            if len(body) >= 2 and not first_blank:
                out.insert(i + 1, '')
                k += 1
            elif len(body) == 1 and first_blank:
                #one-liner: drop blank lines around it
                del out[i + 1]
                k -= 1
                if out[k - 1].strip() == '' and k - 1 > i:
                    del out[k - 1]
            i += 1
            continue
        i += 1
    return out


def fmt(text, path, warnings):
    nl = '\r\n' if '\r\n' in text else '\n'
    lines = text.replace('\r\n', '\n').split('\n')
    lines = join_wrapped(lines, warnings, path)
    lines = split_else(lines)
    lines = method_blank_lines(lines)
    return nl.join(lines)


def main():
    args = sys.argv[1:]
    check = False
    if args and args[0] == '--check':
        check = True
        args = args[1:]
    warnings = []
    changed = 0
    for p in args:
        with open(p, encoding='utf-8', newline='') as f:
            text = f.read()
        new = fmt(text, p, warnings)
        if new != text:
            changed += 1
            print('changed:', p)
            if not check:
                with open(p, 'w', encoding='utf-8', newline='') as f:
                    f.write(new)
    for w in warnings:
        print('warning:', w)
    print('%d file(s) %s' % (changed, 'would change' if check else 'changed'))


if __name__ == '__main__':
    main()
