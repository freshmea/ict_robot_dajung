def sum_all(*value):
    sum = int()
    for i in value:
        sum += i
    avr = sum / len(value)
    return sum, avr

def main():
    # s, a = sum_all(100,28,39,23,41,123,4234,5234,123)
    s = sum_all(100,28,39,23,41,123,4234,5234,123)[0]
    # print(f'합계는: {s}, 평균은: {a}')
    print('합계는:', s)
if __name__ == '__main__':
    main()
