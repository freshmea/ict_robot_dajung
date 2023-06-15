class Student:
    count = int()
    students = list()
    def __init__(self, name, korean, math, english, science):
        self.name = name
        self.korean = korean
        self.math = math
        self.english = english
        self.science = science
        Student.count += 1
        Student.students.append(self)
        print(f'{Student.count} 번째 학생이 생성되었습니다.')

    def get_sum(self):
        return self.korean + self.math + self.english + self.science

    def get_average(self):
        return self.get_sum()/4

    @classmethod
    def print(cls):
        print(f'현재 생성된 총 학생 수는 {Student.count}명 입니다.')
        print()
        print('------------학생 목록------------')
        print('이름\t총점\t평균')
        for student in cls.students:
            print(student)
        print('-------------------------------')

    def __str__(self):
        return f"{self.name}\t{self.get_sum()}\t{self.get_average()}"

    def __gt__(self, value):
        if isinstance(value, Student):
            return self.get_sum() > value.get_sum()
        if isinstance(value, int):
            return self.get_sum() > value

    def __eq__(self, value):
        if isinstance(value, Student):
            return self.get_sum() == value.get_sum()
        if isinstance(value, int):
            return self.get_sum() == value

    def __ne__(self, value):
        if isinstance(value, Student):
            return self.get_sum() != value.get_sum()
        if isinstance(value, int):
            return self.get_sum() != value

    def __ge__(self, value):
        if isinstance(value, Student):
            return self.get_sum() >= value.get_sum()
        if isinstance(value, int):
            return self.get_sum() >= value

    def __lt__(self, value):
        if isinstance(value, Student):
            return self.get_sum() < value.get_sum()
        if isinstance(value, int):
            return self.get_sum() < value

    def __le__(self, value):
        if isinstance(value, Student):
            return self.get_sum() <= value.get_sum()
        if isinstance(value, int):
            return self.get_sum() <= value
        
    def __del__(self):
        print(f'{self.name} 데이터가 소멸 하였습니다.')



def main():
    Student('윤인성', 87, 98, 88, 95)
    Student('연하진', 92, 98, 96, 98)
    Student('구지연', 76, 96, 94, 90)
    Student('나선주', 98, 92, 96, 92)
    Student('윤아린', 95, 98, 98, 98)
    Student('윤명월', 64, 88, 92, 92)
    Student('최수길', 93, 23, 13, 42)
    Student.print()
    
if __name__ == '__main__':
    main()
