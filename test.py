from enum import IntEnum


class Variable(IntEnum):
    THIS = 0b1
    THAT = 0b10
    THAE = 0b100
    THIO = 0b1000


# Variable.THIS = 5
a = Variable(2)
print("a", a)
print(Variable.THIS)
for i in Variable:
    print(i)
    print(type(i))
    if i == 1:
        print("ok")
option = Variable.THAE + Variable.THAT
print(option)
print(option & Variable.THAT == Variable.THIO)
