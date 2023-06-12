def main():
    input_var = input('숫자를 입력하세요:')
    print(type(input_var), input_var)
    # print(input_var + 100)
    print(int(input_var)+100) # 캐스팅 str -> int

if __name__ == '__main__':
    main()
