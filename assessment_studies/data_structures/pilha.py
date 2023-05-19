# Implementação de uma pilha - Last-In, First-Out (LIFO)
# "Fila ingênua" - Não é a melhor forma de implementar uma fila, mas é a mais simples

class Pilha(list): # Classe Pilha herda de uma lista
    
    def __init__(self, sequencia): # Construtor da classe
        # Colocamos, abaixo, um for em uma linha só, chamado de 'list comprehension'
        super().__init__(item for item in sequencia) # 'super', assim como 'self' é uma palavra especial que vê de quem a classe herdou. Serve para chamar o construtor da classe anterior.

    # Métodos de uma pilha

    def append(self, x): # Append serve para adicionar um elemento no final da pilha
        super().append(x)

    def pop(self): # Pop serve para remover o último elemento adicionado à pilha (ou seja, o ÚLTIMO da fila)
        return super().pop()

    def __repr__(self): # Serve para representação quando printamos algo
        return f"Pilha - {super().__repr__()}"


def main():
    f = Pilha([1, 2, 3, 4, 5])
    print(f)
    f.append(6)
    print(f)
    f.pop()
    print(f)

if __name__ == '__main__':
    main() # Encapsulamento de funções