// This code is taken from https://github.com/Cathedrow/Cryptosuite (Peter Knight) and is lightly modified for MySensors use
#ifndef Sha256_h
#define Sha256_h
#if !DOXYGEN
#include <inttypes.h>

#define HASH_LENGTH 32
#define BLOCK_LENGTH 64

union _buffer {
	uint8_t b[BLOCK_LENGTH];
	uint32_t w[BLOCK_LENGTH/4];
};
union _state {
	uint8_t b[HASH_LENGTH];
	uint32_t w[HASH_LENGTH/4];
};

class Sha256Class
{
public:
	Sha256Class(); // Constructor
	void init(void);
	uint8_t* result(void);
	void write(uint8_t);
private:
	void pad();
	void addUncounted(uint8_t data);
	void hashBlock();
	uint32_t ror32(uint32_t number, uint8_t bits);
	_buffer buffer;
	uint8_t bufferOffset;
	_state state;
	uint32_t byteCount;
};

class HmacClass : public Sha256Class
{
public:
	HmacClass(); // Constructor
	void initHmac(const uint8_t* secret, int secretLength);
	uint8_t* resultHmac(void);
private:
	uint8_t keyBuffer[BLOCK_LENGTH];
	uint8_t innerHash[HASH_LENGTH];
};

#endif
#endif
