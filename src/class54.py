class Myerror(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)
        self.args = ['이것은 내가 만든 에러 입니다.']


def main():
    number = input('정수 입력')
    number = int(number)
    try:
        if number < 0 :
            raise Myerror
        else:
            raise NotImplementedError
    except NotImplementedError:
        print('아직 구현 되지 않음')
    except Myerror as e:
        print(e)
        print('내가 만든 에러객체')
        

if __name__ == '__main__':
    main()
