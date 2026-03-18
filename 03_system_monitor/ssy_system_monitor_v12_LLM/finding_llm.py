import google.generativeai as genai

# 서영님의 키 입력
genai.configure(api_key="MY_KEY") 

for m in genai.list_models():
    if 'generateContent' in m.supported_generation_methods:
        print(f"사용 가능 모델명: {m.name}")
