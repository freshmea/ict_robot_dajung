# ict_robot_dajung

## 구글 워크 시트

* 파이썬 : https://docs.google.com/spreadsheets/d/1ZKnZpHw-48zsVkUkT5n6LNv3toct_AxbJAIT7m4TOig/edit?usp=sharing
* OpenCV : https://docs.google.com/spreadsheets/d/1odxo_JU_2nAmZ-tA_Qo6QQWC38ebB5ka0-9BWJsOO7Q/edit?usp=sharing
* ROS2 : https://docs.google.com/spreadsheets/d/1Cjcy4guMlkmEkryRMsu3g7H_shr6skfevS71hDpA7DI/edit?usp=sharing


* history 파일 업데이트 7/2 


* ros2 build 에러가 다음과 같을 때 
  ```
/usr/lib/python3.10/site-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  warnings.warn(
  ```
  이 싸이트를 참고해서 해결 
  https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/

  setuptools 버전을 58.2.0 으로 다운 그래이드 함 
  pip install setuptools==58.2.0
  python3 -m pip install setuptools==58.2.0