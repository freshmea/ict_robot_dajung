def test():
    print('함수가 호출되었습니다.')
    yield 'test'

def main():
    # 실행이 안되는 예제.
    print('A 지점 통과')
    test()
    
    print('B 지점 통과')
    test()
    print(test())

if __name__ == '__main__':
    main()
