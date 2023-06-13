def test():
    print('first line')
    try:
        print('try')
        # raise
        return
        print('after return')
    except:
        print('except')
    else:
        print('else')
    finally:
        print('finally')
    print('end of test')

def main():
    test()

if __name__ == '__main__':
    main()
