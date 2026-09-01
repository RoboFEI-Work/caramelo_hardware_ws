#ifndef CARAMELO_HARDWARE__QUADRATURE_DECODER_HPP_
#define CARAMELO_HARDWARE__QUADRATURE_DECODER_HPP_

// Decodificador de quadratura x4 PURO: sem ROS, sem GPIO, sem alocacao.
// Entrada = uma palavra de 32 bits com os niveis crus das linhas (a mesma que o
// RIO_IN do RP1 entrega numa unica leitura MMIO). Saida = delta de contagens.
//
// POR QUE AMOSTRAGEM E NAO EVENTO (decisao de arquitetura, 2026-09-01):
// medir o SENTIDO exige observar os DOIS canais; observar dois canais POR EVENTO
// custa 4*f eventos/s por roda, o mesmo custo contando x1, x2 ou x4 — ou seja,
// x1 nao economiza nada, so' joga informacao fora. E 4*f em userspace e' o que
// ja falhou em julho de 2026 (438k eventos/s a 6 rad/s, 1.46M/s a 20 rad/s:
// perdia bordas e esfomeava a thread de PWM do lgpio). Amostrando, o custo e'
// CONSTANTE: independe da velocidade da roda E do chilrear do encoder parado.
// Medido nesta Pi 5: 6.0-6.4 MHz de leitura do RIO_IN (~160 ns por leitura),
// contra 2.73 us de intervalo minimo entre transicoes na velocidade maxima da
// roda -> ~16 amostras por transicao.
//
// POR QUE ISSO MATA O FANTASMA DO ENCODER PARADO:
// no codigo Gray 00 -> 01 -> 11 -> 10, uma linha oscilando SOZINHA (o chilrear
// do comparador com a roda parada em cima de uma borda optica, ~60 kHz medidos)
// produz +1, -1, +1, -1... A soma e' identicamente ZERO e o erro instantaneo
// fica limitado a +-1 count = +-2.7 um de arco. Compare com o fantasma da
// decodificacao 1x com sentido vindo do comando: 13 rad/s com o robo PARADO.
// Razao ~2400:1. E, ao contrario do "portao de repouso" que foi revertido em
// f23f5dc, isso nao custa a inercia pos-comando nem o empurrao manual.

#include <array>
#include <cstddef>
#include <cstdint>

namespace caramelo
{

/// Estado de quadratura codificado como (A << 1) | B, na ordem Gray 0,1,3,2.
/// Marcador de transicao ilegal (salto diagonal 00<->11 ou 01<->10).
inline constexpr int8_t kQuadIllegal = 2;

/// Tabela de transicao x4 indexada por (anterior << 2) | atual.
/// +1/-1 = um passo; 0 = sem mudanca; kQuadIllegal = salto diagonal.
inline constexpr std::array<int8_t, 16> kQuadTable = {
	//        cur=00           cur=01           cur=10           cur=11
	/*00*/ 0, 1, -1, kQuadIllegal,
	/*01*/ -1, 0, kQuadIllegal, 1,
	/*10*/ 1, kQuadIllegal, 0, -1,
	/*11*/ kQuadIllegal, -1, 1, 0,
};

/// Delta de um passo de quadratura. Retorna kQuadIllegal na diagonal.
inline constexpr int8_t quad_step(uint8_t prev_state, uint8_t cur_state)
{
	return kQuadTable[static_cast<std::size_t>((prev_state & 0x3u) << 2 | (cur_state & 0x3u))];
}

/// Mapeamento de uma roda para dois bits da palavra amostrada.
struct ChannelMap
{
	uint8_t a_bit = 0;   ///< indice do bit do canal A FISICO na palavra
	uint8_t b_bit = 0;   ///< indice do bit do canal B FISICO na palavra
	int8_t sign = 1;     ///< +1/-1: converte o sentido decodificado para o sentido da junta
};

/// Decodificador de N rodas alimentado por palavras de niveis crus.
///
/// Uso: `reset(primeira_palavra)` uma vez, depois `update(palavra)` a cada
/// amostra. `update` e' o caminho critico — sem syscall, sem alocacao, sem lock.
template<std::size_t N>
class QuadratureDecoder
{
public:
	/// \param channels mapeamento de bits e sinal por roda
	/// \param stable_samples quantas amostras consecutivas uma palavra precisa
	///        durar para ser aceita. 1 = sem filtro. Ver comentario de commit().
	explicit QuadratureDecoder(
		const std::array<ChannelMap, N> & channels, uint32_t stable_samples = 1)
	: channels_(channels), stable_(stable_samples < 1 ? 1 : stable_samples)
	{
		// Mascara das linhas que nos interessam: o caminho rapido de update()
		// so' faz trabalho de verdade quando ALGUM desses bits mudou.
		for (const auto & ch : channels_) {
			mask_ |= (1u << ch.a_bit) | (1u << ch.b_bit);
		}
	}

	/// Sincroniza o estado interno com a palavra atual, sem contar nada.
	void reset(uint32_t word)
	{
		last_word_ = word & mask_;
		run_len_ = 1;
		for (std::size_t i = 0; i < N; ++i) {
			states_[i] = state_of(last_word_, channels_[i]);
			counts_[i] = 0;
			illegal_[i] = 0;
			rejected_ = 0;
		}
	}

	/// Processa uma amostra. Retorna true se a palavra observada mudou.
	///
	/// O caminho rapido (nada mudou) e' um AND, um compare e um incremento: na
	/// taxa de amostragem usada, a esmagadora maioria das amostras cai nele.
	inline bool update(uint32_t word)
	{
		const uint32_t masked = word & mask_;
		if (masked == last_word_) {
			++run_len_;
			return false;
		}
		// A palavra ANTERIOR acabou de fechar seu tempo de permanencia. So'
		// entao decidimos se ela era real ou glitch — por isso o commit vem
		// com um "run" de atraso.
		if (run_len_ >= stable_) {
			commit(last_word_);
		} else {
			++rejected_;
		}
		last_word_ = masked;
		run_len_ = 1;
		return true;
	}

	int64_t count(std::size_t i) const { return counts_[i]; }
	uint64_t illegal(std::size_t i) const { return illegal_[i]; }
	uint8_t state(std::size_t i) const { return states_[i]; }
	uint32_t mask() const { return mask_; }
	uint32_t stable_samples() const { return stable_; }
	/// Palavras descartadas por nao durarem `stable_samples` — glitches filtrados.
	uint64_t rejected() const { return rejected_; }

	void zero()
	{
		for (std::size_t i = 0; i < N; ++i) {
			counts_[i] = 0;
			illegal_[i] = 0;
		}
		rejected_ = 0;
	}

private:
	/// Aplica a transicao para uma palavra ja considerada estavel.
	///
	/// FILTRO DE GLITCH POR PERMANENCIA: uma palavra so' e' aceita se durou
	/// `stable_` amostras consecutivas. Medido nesta Pi em 2026-09-01, girando
	/// as rodas a mao: com stable_=1 a roda BR acumulou 1709 transicoes ILEGAIS
	/// (as duas linhas mudando dentro da mesma janela de 184 ns) em ~332 mil
	/// counts. Isso NAO e' perda de amostragem — a 5.4 MHz ha ~124 amostras por
	/// transicao legitima na velocidade de mao. E' qualidade de sinal: ringing e
	/// diafonia entre os fios A e B do mesmo chicote, exatamente na roda que a
	/// bancada de julho ja tinha apontado como a pior (15304 descidas duplas em
	/// 15516 ciclos). Cada ilegal custa 2 counts de erro, entao 1709 delas
	/// explicam ~1% do deficit medido nessa roda.
	///
	/// A escolha de `stable_` tem um teto fisico: na velocidade maxima de roda
	/// (20.06 rad/s) o intervalo minimo entre transicoes legitimas e' 2.73 us,
	/// ou ~15 amostras a 5.4 MHz. Manter stable_ bem abaixo disso (4 a 8) filtra
	/// glitches de ate ~1.5 us sem tocar em borda de verdade.
	inline void commit(uint32_t stable_word)
	{
		for (std::size_t i = 0; i < N; ++i) {
			const uint8_t cur = state_of(stable_word, channels_[i]);
			const uint8_t prev = states_[i];
			if (cur == prev) {
				continue;
			}
			states_[i] = cur;
			const int8_t step = quad_step(prev, cur);
			if (step == kQuadIllegal) {
				// Salto diagonal entre dois estados ESTAVEIS: nao da para
				// inferir o sentido. Conta zero e denuncia no contador.
				++illegal_[i];
				continue;
			}
			counts_[i] += static_cast<int64_t>(step) * channels_[i].sign;
		}
	}

	static inline uint8_t state_of(uint32_t word, const ChannelMap & ch)
	{
		const uint8_t a = static_cast<uint8_t>((word >> ch.a_bit) & 1u);
		const uint8_t b = static_cast<uint8_t>((word >> ch.b_bit) & 1u);
		return static_cast<uint8_t>((a << 1) | b);
	}

	std::array<ChannelMap, N> channels_{};
	std::array<uint8_t, N> states_{};
	std::array<int64_t, N> counts_{};
	std::array<uint64_t, N> illegal_{};
	uint64_t rejected_ = 0;
	uint32_t last_word_ = 0;
	uint32_t run_len_ = 0;
	uint32_t stable_ = 1;
	uint32_t mask_ = 0;
};

}  // namespace caramelo

#endif  // CARAMELO_HARDWARE__QUADRATURE_DECODER_HPP_
