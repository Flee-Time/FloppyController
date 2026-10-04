"""Execute actual pioasm opcodes and check cycles, polarity and termination."""
import collections
import pathlib
import random
import subprocess
import sys
import tempfile
import unittest

with tempfile.TemporaryDirectory() as tmp:
    output = pathlib.Path(tmp) / "writer.hex"
    subprocess.run([sys.argv[1], "-o", "hex", sys.argv[2], str(output)], check=True)
    PROGRAM = [int(line, 16) for line in output.read_text().splitlines()]


def execute(words, bit_count, truncate=False):
    fifo = collections.deque([bit_count - 1] + (words[:-1] if truncate else words))
    pc = 0
    osr = 0
    shifted = 32
    x = y = 0
    cycles = 0
    pin = 1
    edges = []
    bits = []
    while cycles < bit_count * 40 + 100:
        insn = PROGRAM[pc]
        opcode = insn >> 13
        delay = (insn >> 8) & 31
        arg = insn & 255
        next_pc = pc + 1
        if opcode != 3 and shifted == 32 and fifo:
            osr = fifo.popleft()
            shifted = 0
        if opcode == 4:  # PULL BLOCK (autopull fence if OSR already full)
            if shifted == 32:
                if not fifo:
                    return bits, edges, cycles, pin, False
                osr = fifo.popleft()
                shifted = 0
        elif opcode == 3:  # OUT, left-shifting OSR
            if shifted == 32:
                if not fifo:
                    return bits, edges, cycles, pin, False
                osr = fifo.popleft()
                shifted = 0
            count = (arg & 31) or 32
            value = osr >> (32 - count)
            osr = (osr << count) & 0xffffffff
            shifted += count
            if (arg >> 5) == 1:
                x = value
                bits.append((cycles, value))
            elif (arg >> 5) == 2:
                y = value
            else:
                raise AssertionError("unexpected OUT destination")
            if shifted == 32 and fifo:
                osr = fifo.popleft()
                shifted = 0
        elif opcode == 0:  # JMP
            condition = (arg >> 5) & 7
            take = condition == 0 or (condition == 1 and x == 0)
            if condition == 4:  # Y--: test before decrement, including zero
                take = y != 0
                y = (y - 1) & 0xffffffff
            if take:
                next_pc = arg & 31
        elif opcode == 7:  # SET PINS
            new_pin = arg & 1
            if pin != new_pin:
                edges.append((cycles, new_pin))
            pin = new_pin
        elif opcode == 5:  # MOV Y,Y (NOP)
            assert arg == 0x42
        elif opcode == 6:  # IRQ 0 REL
            assert arg == 0x10
            return bits, edges, cycles, pin, True
        else:
            raise AssertionError(f"unexpected instruction {insn:04x}")
        cycles += delay + 1
        pc = next_pc
    raise AssertionError("PIO failed to terminate")


class TimingTests(unittest.TestCase):
    def check_stream(self, words):
        n = len(words) * 32
        bits, edges, cycles, pin, complete = execute(words, n)
        expected = [(word >> bit) & 1 for word in words for bit in range(31, -1, -1)]
        self.assertTrue(complete)
        self.assertEqual([value for _, value in bits], expected)
        self.assertEqual([t for t, _ in bits], list(range(2, 2 + n * 20, 20)))
        self.assertEqual(cycles, 2 + n * 20)
        self.assertEqual(pin, 1)
        lows = [t for t, value in edges if value == 0]
        highs = [t for t, value in edges if value == 1]
        self.assertEqual(lows, [2 + i * 20 + 2 for i, bit in enumerate(expected) if bit])
        self.assertEqual([high - low for low, high in zip(lows, highs)], [4] * len(lows))
        # 20 MHz: 20 cycles = 1 us, LOW = 200 ns, independently of data.

    def test_both_branches_and_word_boundaries(self):
        for words in ([0], [0xffffffff], [0x44894489], [0x92499249],
                      [0x80000001, 0x44895555, 0xaaaaaaaa],
                      [0xaaaaaaaa] * 266):
            self.check_stream(words)

    def test_random_payloads(self):
        rng = random.Random(0x4489)
        for _ in range(32):
            self.check_stream([rng.getrandbits(32) for _ in range(266)])

    def test_underrun_never_signals_completion(self):
        bits, _, _, pin, complete = execute([0x44894489] * 4, 128, truncate=True)
        self.assertFalse(complete)
        self.assertEqual(len(bits), 96)
        self.assertEqual(pin, 1)

    def test_exact_last_half_cell(self):
        for n in (1, 31, 32, 33, 64):
            bits, _, cycles, pin, complete = execute([0xffffffff] * ((n + 31) // 32), n)
            self.assertTrue(complete)
            self.assertEqual(len(bits), n)
            self.assertEqual(cycles, 2 + n * 20)
            self.assertEqual(pin, 1)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
