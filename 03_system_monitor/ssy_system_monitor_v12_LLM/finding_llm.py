import google.generativeai as genai

# 서영님의 키 입력
genai.configure(api_key="AIzaSyAUxxmUwxB1rR_jFob0zIp56I1Q5vquXJ4") 

for m in genai.list_models():
    if 'generateContent' in m.supported_generation_methods:
        print(f"사용 가능 모델명: {m.name}")