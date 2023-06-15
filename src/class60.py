from urllib import request

def main():
    target = request.urlopen('http://www.hanbit.co.kr/images/common/logo_hanbit.png')
    output = target.read()
    with open('data/output.png', 'wb') as f:
        f.write(output)

if __name__ == '__main__':
    main()
