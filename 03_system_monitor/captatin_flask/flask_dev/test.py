import gradio as gr

# 완강기 사용법 안내문
instructions = {
    ("어린이", "한국어"): "🧒 어린이 위한 완강기 사용법:\n\n1. 어른의 도움을 받아 완강기를 착용하세요.\n2. 팔을 만세하고 벨트를 겨드랑이 밑에 매요.\n3. 벨트가 빠지지 않도록 꽉 조이면 안전하게 내려갈 수 있어요.\n4. 창 밖으로 발을 내밀어서 내려갈 준비를 해요.\n5. 두 손으로 벽을 짚으며 천천히 내려가요.\n6. 땅에 도착한 다음 벨트를 풀면 줄이 알아서 올라가요.\n\n🔥 항상 어른의 안내를 따라요.",
    ("성인", "한국어"): "👴 성인을 위한 완강기 사용법:\n\n1. 완강기를 튼튼하게 고정하세요.\n2. 안전벨트를 겨드랑이 밑에 조여 착용합니다.\n3. 창 밖으로 몸을 내밀어 손으로 벽을 짚으며 천천히 하강합니다.\n4. 지상 도착 후 벨트를 풀면 자동으로 되감이 됩니다.\n\n📢 주변 사람에게도 사용법을 알려주세요!",
    ("노인", "한국어"): "👵 노인을 위한 완강기 사용법:\n\n1. 주변 사람이 도와주는 것이 안전합니다.\n2. 안전벨트를 겨드랑이 밑에 조여 착용합니다.\n3. 창 밖으로 몸을 내밀어 손으로 벽을 짚으며 천천히 하강합니다.\n4. 지상 도착 후 필요하면 구조대의 도움을 기다리세요.",

    ("어린이", "영어"): "🧒 For Children:\n\n1. Put on the rope ladder with the help of an adult.\n2. Raise your arms and place the belt under your armpits.\n3. Tighten the belt so that it doesn't slip off, and you can go down safely.\n4. Put your feet out the window and prepare to go down.\n5. Grasp the wall with both hands and slowly go down.\n6. Once you reach the ground, release the belt and the rope will go up on its own.\n\n🔥 Always follow the guidance of an adult.",
    ("성인", "영어"): "👴 For Adults:\n\n1. Fasten the slinger firmly.\n2. Fasten the seat belt under your armpit.\n3. Lean out of the window and slowly descend while supporting yourself with your hands against the wall.\n4. Once you reach the ground, release the belt and it will automatically rewind.\n\n📢 Teach the people around you how to use it!",
    ("노인", "영어"): "👵 For Elderly:\n\n1. It is safer to have someone around you help you.\n2. Fasten your seat belt under your armpit.\n3. Lean out of the window, support yourself with your hands against the wall, and slowly descend.\n4. After reaching the ground, wait for rescue assistance if necessary.",

    ("어린이", "중국어"): "🧒 儿童使用说明：\n\n1. 在成人的帮助下戴上支架。\n2. 保持手臂抬起，并将腰带放在腋下。\n3. 只要系紧安全带不让它脱落，就能安全下来。\n4. 把脚伸出窗外并准备爬下来。\n5. 双手支撑身体，慢慢沿墙往走。\n6. 一旦到达地面，松开安全带，绳子就会自动升起。\n\n🔥 始终遵循成人的指导。",
    ("성인", "중국어"): "👴 成人使用说明：\n\n1. 牢固固定支架。\n2. 将安全带系在腋下。\n3. 探出窗外，用手支撑自己靠在墙上，然后慢慢下降。\n4. 到达地面后，松开皮带，它就会自动回卷。\n\n📢 请也教会你周围的人如何使用它！",
    ("노인", "중국어"): "👵 老年人使用说明：\n\n1. 身边有人帮助你会更安全。\n2. 将安全带系在腋下。\n3. 将身体探出窗外，双手扶着墙壁支撑身体，慢慢下降。\n4. 到达地面后，如有需要，等待救援援助。"
}

# 안내문 반환 함수 및 TTS 설정점
from gtts import gTTS
import tempfile

def get_instruction(age, lang):
    text = instructions.get((age, lang), "! 안내문을 찾을 수 없습니다. 다시 선택해주세요.")
    audio = None
    if age == "노인":
        # 텍스트를 mp3로 변환
        tts = gTTS(text, lang='ko' if lang == "한국어" else 'en' if lang == "영어" else 'zh-CN')
        temp_audio = tempfile.NamedTemporaryFile(delete=False, suffix=".mp3")
        tts.save(temp_audio.name)
        audio = temp_audio.name # 파일 경로 반환
    return text, audio

# 문자 신고용 텍스트
emergency_text = "화재가 발생했습니다. 현재 위치는 [단국대학교 1공학관 5층]입니다. 구조가 필요합니다."

# Gradio UI 구성
with gr.Blocks() as demo:
    gr.Markdown("## 🚒 완강기 사용 안내 시스템")

    # 탭 1: 문자 신고 / 맞춤형 안내
    with gr.Tab("📍 119 문자 신고"):
        gr.Markdown("**버튼을 누르면 문자창에 내용이 자동 입력됩니다.**")
        gr.Textbox(value=emergency_text, label="문자 내용", interactive=False)
        gr.HTML('<a href="sms:119?body=화재가 발생했습니다. 현재 위치는 [단국대학교 1공학관 5층]입니다. 구조가 필요합니다." target="_blank"><button style="padding:10px; font-size:16px;">🚨 문자 신고하기</button></a>')

    with gr.Tab("💡 맞춤형 대피 안내"):
        age = gr.Dropdown(["어린이", "성인", "노인"], label="연령 선택")
        lang = gr.Dropdown(["한국어", "영어", "중국어"], label="언어 선택")
        run_btn = gr.Button("📋 대피 안내 받기")
        output = gr.Textbox(label="안내문", lines=12)
        tts = gr.Audio(label="🔊 음성 안내 (노인 선택 시)", interactive=False)

        run_btn.click(get_instruction, inputs=[age, lang], outputs=[output, tts])

# 실행
demo.launch(share=True)
import gradio as gr

# 완강기 사용법 안내문
instructions = {
    ("어린이", "한국어"): "🧒 어린이 위한 완강기 사용법:\n\n1. 어른의 도움을 받아 완강기를 착용하세요.\n2. 팔을 만세하고 벨트를 겨드랑이 밑에 매요.\n3. 벨트가 빠지지 않도록 꽉 조이면 안전하게 내려갈 수 있어요.\n4. 창 밖으로 발을 내밀어서 내려갈 준비를 해요.\n5. 두 손으로 벽을 짚으며 천천히 내려가요.\n6. 땅에 도착한 다음 벨트를 풀면 줄이 알아서 올라가요.\n\n🔥 항상 어른의 안내를 따라요.",
    ("성인", "한국어"): "👴 성인을 위한 완강기 사용법:\n\n1. 완강기를 튼튼하게 고정하세요.\n2. 안전벨트를 겨드랑이 밑에 조여 착용합니다.\n3. 창 밖으로 몸을 내밀어 손으로 벽을 짚으며 천천히 하강합니다.\n4. 지상 도착 후 벨트를 풀면 자동으로 되감이 됩니다.\n\n📢 주변 사람에게도 사용법을 알려주세요!",
    ("노인", "한국어"): "👵 노인을 위한 완강기 사용법:\n\n1. 주변 사람이 도와주는 것이 안전합니다.\n2. 안전벨트를 겨드랑이 밑에 조여 착용합니다.\n3. 창 밖으로 몸을 내밀어 손으로 벽을 짚으며 천천히 하강합니다.\n4. 지상 도착 후 필요하면 구조대의 도움을 기다리세요.",

    ("어린이", "영어"): "🧒 For Children:\n\n1. Put on the rope ladder with the help of an adult.\n2. Raise your arms and place the belt under your armpits.\n3. Tighten the belt so that it doesn't slip off, and you can go down safely.\n4. Put your feet out the window and prepare to go down.\n5. Grasp the wall with both hands and slowly go down.\n6. Once you reach the ground, release the belt and the rope will go up on its own.\n\n🔥 Always follow the guidance of an adult.",
    ("성인", "영어"): "👴 For Adults:\n\n1. Fasten the slinger firmly.\n2. Fasten the seat belt under your armpit.\n3. Lean out of the window and slowly descend while supporting yourself with your hands against the wall.\n4. Once you reach the ground, release the belt and it will automatically rewind.\n\n📢 Teach the people around you how to use it!",
    ("노인", "영어"): "👵 For Elderly:\n\n1. It is safer to have someone around you help you.\n2. Fasten your seat belt under your armpit.\n3. Lean out of the window, support yourself with your hands against the wall, and slowly descend.\n4. After reaching the ground, wait for rescue assistance if necessary.",

    ("어린이", "중국어"): "🧒 儿童使用说明：\n\n1. 在成人的帮助下戴上支架。\n2. 保持手臂抬起，并将腰带放在腋下。\n3. 只要系紧安全带不让它脱落，就能安全下来。\n4. 把脚伸出窗外并准备爬下来。\n5. 双手支撑身体，慢慢沿墙往走。\n6. 一旦到达地面，松开安全带，绳子就会自动升起。\n\n🔥 始终遵循成人的指导。",
    ("성인", "중국어"): "👴 成人使用说明：\n\n1. 牢固固定支架。\n2. 将安全带系在腋下。\n3. 探出窗外，用手支撑自己靠在墙上，然后慢慢下降。\n4. 到达地面后，松开皮带，它就会自动回卷。\n\n📢 请也教会你周围的人如何使用它！",
    ("노인", "중국어"): "👵 老年人使用说明：\n\n1. 身边有人帮助你会更安全。\n2. 将安全带系在腋下。\n3. 将身体探出窗外，双手扶着墙壁支撑身体，慢慢下降。\n4. 到达地面后，如有需要，等待救援援助。"
}

# 안내문 반환 함수 및 TTS 설정점
from gtts import gTTS
import tempfile

def get_instruction(age, lang):
    text = instructions.get((age, lang), "! 안내문을 찾을 수 없습니다. 다시 선택해주세요.")
    audio = None
    if age == "노인":
        # 텍스트를 mp3로 변환
        tts = gTTS(text, lang='ko' if lang == "한국어" else 'en' if lang == "영어" else 'zh-CN')
        temp_audio = tempfile.NamedTemporaryFile(delete=False, suffix=".mp3")
        tts.save(temp_audio.name)
        audio = temp_audio.name # 파일 경로 반환
    return text, audio

# 문자 신고용 텍스트
emergency_text = "화재가 발생했습니다. 현재 위치는 [단국대학교 1공학관 5층]입니다. 구조가 필요합니다."

# Gradio UI 구성
with gr.Blocks() as demo:
    gr.Markdown("## 🚒 완강기 사용 안내 시스템")

    # 탭 1: 문자 신고 / 맞춤형 안내
    with gr.Tab("📍 119 문자 신고"):
        gr.Markdown("**버튼을 누르면 문자창에 내용이 자동 입력됩니다.**")
        gr.Textbox(value=emergency_text, label="문자 내용", interactive=False)
        gr.HTML('<a href="sms:119?body=화재가 발생했습니다. 현재 위치는 [단국대학교 1공학관 5층]입니다. 구조가 필요합니다." target="_blank"><button style="padding:10px; font-size:16px;">🚨 문자 신고하기</button></a>')

    with gr.Tab("💡 맞춤형 대피 안내"):
        age = gr.Dropdown(["어린이", "성인", "노인"], label="연령 선택")
        lang = gr.Dropdown(["한국어", "영어", "중국어"], label="언어 선택")
        run_btn = gr.Button("📋 대피 안내 받기")
        output = gr.Textbox(label="안내문", lines=12)
        tts = gr.Audio(label="🔊 음성 안내 (노인 선택 시)", interactive=False)

        run_btn.click(get_instruction, inputs=[age, lang], outputs=[output, tts])

# 실행
demo.launch(share=True)