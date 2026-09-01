#ifndef CARAMELO_HARDWARE__RP1_RIO_HPP_
#define CARAMELO_HARDWARE__RP1_RIO_HPP_

// Acesso direto ao bloco RIO do RP1 (Raspberry Pi 5) por MMIO em /dev/gpiomem0.
//
// UMA leitura de 32 bits devolve o nivel de GPIO0..27 SIMULTANEAMENTE — ou seja,
// os 8 canais de encoder do robo numa unica transacao. E' isso que torna a
// decodificacao de quadratura por amostragem barata o bastante para substituir a
// contagem por evento (ver quadrature_decoder.hpp para o argumento completo).
//
// Requisitos na Pi (cobertos por tools/setup_pi.sh):
//   - /dev/gpiomem0 acessivel ao grupo gpio (regra udev 99-caramelo-gpiomem);
//     o Ubuntu, ao contrario do Raspberry Pi OS, nao entrega isso por padrao.
//   - Sem root: basta pertencer ao grupo gpio.
//
// Medido nesta Pi 5 (Ubuntu 24.04, kernel 6.8.0-1060-raspi): 6.0-6.4 MHz de
// leituras sustentadas de RIO_IN, ~160 ns por leitura.

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

namespace caramelo
{

/// Janela que /dev/gpiomem0 expoe (mesmo layout que o utilitario pinctrl usa).
/// O mmap SO aceita este tamanho — pedir 4 KiB devolve EINVAL.
inline constexpr std::size_t kRp1MapSize = 0x30000u;
inline constexpr std::size_t kRp1IoBank0 = 0x00000u;
inline constexpr std::size_t kRp1SysRio0 = 0x10000u;
inline constexpr std::size_t kRp1PadsBank0 = 0x20000u;

inline constexpr std::size_t kRioOut = 0x00u;
inline constexpr std::size_t kRioOe = 0x04u;
inline constexpr std::size_t kRioIn = 0x08u;

/// FUNCSEL do CTRL de io_bank0 que liga o pino ao RIO (equivalente ao SIO).
inline constexpr uint32_t kFuncselRio = 5u;

/// Bits do registrador de pad (pads_bank0).
inline constexpr uint32_t kPadSlewfast = 1u << 0;
inline constexpr uint32_t kPadSchmitt = 1u << 1;
inline constexpr uint32_t kPadPde = 1u << 2;   ///< pull-down enable
inline constexpr uint32_t kPadPue = 1u << 3;   ///< pull-up enable
inline constexpr uint32_t kPadIe = 1u << 6;    ///< input enable
inline constexpr uint32_t kPadOd = 1u << 7;    ///< output disable

/// Mapeamento do RP1. Nao copiavel: e' dono do mmap.
class Rp1Rio
{
public:
	Rp1Rio() = default;
	Rp1Rio(const Rp1Rio &) = delete;
	Rp1Rio & operator=(const Rp1Rio &) = delete;
	~Rp1Rio() { close_all(); }

	/// Abre e mapeia. Devolve string vazia em caso de sucesso, ou o erro.
	std::string open_device(const char * path = "/dev/gpiomem0")
	{
		close_all();
		fd_ = ::open(path, O_RDWR | O_SYNC);
		if (fd_ < 0) {
			return std::string("open ") + path + ": " + std::strerror(errno) +
			       " (o usuario esta no grupo gpio? a regra udev 99-caramelo-gpiomem existe?)";
		}
		void * p = ::mmap(nullptr, kRp1MapSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
		if (p == MAP_FAILED) {
			const std::string err = std::string("mmap: ") + std::strerror(errno);
			close_all();
			return err;
		}
		base_ = static_cast<volatile uint8_t *>(p);
		rio_in_ = reg(kRp1SysRio0 + kRioIn);
		return {};
	}

	bool is_open() const { return base_ != nullptr; }

	/// Leitura crua dos niveis de GPIO0..27. Caminho critico do decodificador.
	inline uint32_t read_in() const { return *rio_in_; }

	/// Configura um pino como entrada ligada ao RIO, com pull-up e Schmitt.
	///
	/// Sem isso a linha pode ficar em outra funcao (ou com o input desabilitado)
	/// e o RIO_IN nao reflete o pino. O Schmitt importa: as bordas do encoder
	/// chegam com ringing, e histerese no pad e' de graca.
	void configure_input(unsigned gpio, bool pull_up = true, bool schmitt = true)
	{
		// 1) pad: entrada habilitada, saida desabilitada, pull configurado.
		volatile uint32_t * pad = reg(kRp1PadsBank0 + 0x04u + 4u * gpio);
		uint32_t v = *pad;
		v |= kPadIe | kPadOd;
		v &= ~(kPadPue | kPadPde | kPadSchmitt);
		if (pull_up) { v |= kPadPue; }
		if (schmitt) { v |= kPadSchmitt; }
		*pad = v;

		// 2) funcao do pino = RIO.
		volatile uint32_t * ctrl = reg(kRp1IoBank0 + 0x04u + 8u * gpio);
		uint32_t c = *ctrl;
		c = (c & ~0x1fu) | kFuncselRio;
		*ctrl = c;

		// 3) RIO: direcao de entrada (limpa o bit de output enable).
		volatile uint32_t * oe = reg(kRp1SysRio0 + kRioOe);
		*oe = *oe & ~(1u << gpio);
	}

	uint32_t read_pad(unsigned gpio) const
	{
		return *reg(kRp1PadsBank0 + 0x04u + 4u * gpio);
	}

	uint32_t read_ctrl(unsigned gpio) const
	{
		return *reg(kRp1IoBank0 + 0x04u + 8u * gpio);
	}

	uint32_t read_oe() const { return *reg(kRp1SysRio0 + kRioOe); }

private:
	volatile uint32_t * reg(std::size_t off) const
	{
		return reinterpret_cast<volatile uint32_t *>(const_cast<volatile uint8_t *>(base_) + off);
	}

	void close_all()
	{
		if (base_ != nullptr) {
			::munmap(const_cast<uint8_t *>(base_), kRp1MapSize);
			base_ = nullptr;
			rio_in_ = nullptr;
		}
		if (fd_ >= 0) {
			::close(fd_);
			fd_ = -1;
		}
	}

	int fd_ = -1;
	volatile uint8_t * base_ = nullptr;
	volatile uint32_t * rio_in_ = nullptr;
};

}  // namespace caramelo

#endif  // CARAMELO_HARDWARE__RP1_RIO_HPP_
