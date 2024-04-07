#ifndef Print_h
#define Print_h

//#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

struct Print {
	FILE* stream;
	Print(FILE *f) : stream(f) {}

	size_t print(const char *str) {
		return fprintf(stream, "%s", str);
	}

	size_t println(const char *str) {
		return fprintf(stream, "%s\n", str);
	}

	size_t print(char ch) {
		return fprintf(stream, "%c", ch);
	}

	size_t print(int val, int type = DEC) {
		switch (type) {
		case OCT:
			return fprintf(stream, "%o", val);
		case DEC:
			return fprintf(stream, "%d", val);
		case HEX:
			return fprintf(stream, "%x", val);
		}
		return fprintf(stream, "!!!");
	}

	size_t println(int val, int type = DEC) {
		int ret = print(val, type);
		fputc('\n', stream);
		return ret + 1;
	}

	size_t println() {
		fputc('\n', stream);
		return 1;
	}

#define X(T) \
	size_t print(T val, int type = DEC) { return print((int)val, type); } \
	size_t println(T val, int type = DEC) { return println((int)val, type); }
	X(unsigned char)
	X(unsigned int)
#undef X

	void flush() { fflush(stream); }
};
#endif
