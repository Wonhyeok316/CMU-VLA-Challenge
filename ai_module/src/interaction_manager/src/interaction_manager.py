#!/usr/bin/env python
import rospy
import std_msgs.msg
import json
import sys
import os
import hydra

# LEO 관련 import 해오기
current_dir = os.path.dirname(os.path.abspath(__file__))
target_path = os.path.join(current_dir, '..', '..', 'leo_vlm', 'embodied-generalist')
sys.path.append(target_path)

try:
    from inference import LeoProber
except ImportError as e:
    rospy.logwarn(f"Could not import LeoProber: {e}")
    LeoProber = None

class InteractionManager:
    def __init__(self):
        rospy.init_node('interaction_manager', anonymous=True)
        self.question_type_pub = rospy.Publisher('/question_type', std_msgs.msg.String, queue_size=10)
        self.leo_prober = None  # 싱글톤 인스턴스
        self.leo_config = None  # 설정 캐싱
        self.initialize_leo()
        self.question_types = {
            'numerical': 'numerical_question',
            'object': 'object_reference_question',
            'waypoint': 'waypoint_generation_question'
        }

        rospy.loginfo("Interaction Manager initialized. Awaiting questions...")
    
    def initialize_leo(self):
        try:
            # LeoProber는 사용자 질문이 있을 때마다 새로 생성하므로
            # 여기서는 초기화만 확인
            if LeoProber is not None:
                rospy.loginfo("LEO model can be initialized successfully")
            else:
                rospy.logwarn("LeoProber not available")
        except Exception as e:
            rospy.logerr(f"Failed to initialize LEO model: {e}")
            self.leo_prober = None
    def classify_question_type(self, question):
        """Use LEO to classify the question type"""
        if LeoProber is None:
            rospy.logwarn("LEO model not available, using fallback classification")
            return self.fallback_classification(question)
        
        try:
            # 사용자 질문으로 새로운 LeoProber 인스턴스 생성
            result = self.run_leo_inference(question)
            
            # 결과를 기반으로 질문 타입 분류
            question_type = self.parse_leo_result(result)
            return question_type
            
        except Exception as e:
            rospy.logerr(f"LEO inference failed: {e}. Fallback_classification will be used.")
            return self.fallback_classification(question)

    def get_leo_prober(self):
        """싱글톤 패턴으로 LeoProber 인스턴스 반환"""
        try:
            # LeoProber가 이미 존재하면 재사용
            if self.leo_prober is not None:
                rospy.loginfo("Reusing existing LeoProber instance...")
                return self.leo_prober
            
            # 처음 생성할 때만 설정
            config_dir = os.path.join(target_path, 'configs')
            with hydra.initialize_config_dir(version_base=None, config_dir=config_dir):
                # 상대경로 계산 (이식성을 위해)
                current_dir = os.path.dirname(os.path.abspath(__file__))
                ckpt_relative_path = os.path.join(current_dir, '..', '..', 'leo_vlm', 'embodied-generalist', 'results', 'sft_noact')
                ckpt_absolute_path = os.path.abspath(ckpt_relative_path)
                
                cfg = hydra.compose(config_name='default', overrides = [
                    'note=tuning_noact',
                    f'pretrained_ckpt_path={ckpt_absolute_path}',
                    'base_dir=./results',
                    'probe.sources=[text_only]',
                    'probe.scene_ids=[scene_0]',
                    'probe.situations=[You are in a room]',
                    'probe.instructions=["Describe the scene in as much detail as possible."]'  # 기본 질문
                ])
                
                # LeoProber 인스턴스 생성 (한 번만)
                rospy.loginfo("Creating new LeoProber instance...")
                self.leo_prober = LeoProber(cfg)
            
            return self.leo_prober
            
        except Exception as e:
            rospy.logerr(f"Error in get_leo_prober: {e}")
            raise e

    def run_leo_inference(self, question):
        try:
            # 입력 텍스트 정리 (surrogate 유니코드 문자만 제거)
            cleaned_question = question.encode('utf-8', errors='ignore').decode('utf-8').strip()
            if not cleaned_question:
                rospy.logwarn("No question provided, using default question")
                cleaned_question = "What is this?"
            
            # 싱글톤 인스턴스 가져오기
            prober = self.get_leo_prober()
            
                        # 질문 타입 분류를 위한 프롬프트 생성
            classification_prompt = f"""TASK: Classify the question type.

QUESTION: "{cleaned_question}"

OPTIONS:
- numerical_question: for counting, measuring, quantities (how many, count, number, etc.)
- waypoint_generation_question: for navigation, movement, directions (go to, move, navigate, etc.)
- object_reference_question: for describing, identifying objects (what is, describe, identify, etc.)

RESPONSE FORMAT: Respond with ONLY the type name (numerical_question, waypoint_generation_question, or object_reference_question).

ANSWER:"""
                        
            # 분류 프롬프트를 질문으로 설정
            prober.instructions = [classification_prompt]
            
            # 상황 설정도 업데이트
            prober.situations = ["You need to classify the question type."]
            
            # 디버깅: 설정값 확인
            rospy.loginfo(f"DEBUG: prober.instructions = {prober.instructions}")
            rospy.loginfo(f"DEBUG: prober.situations = {prober.situations}")
            rospy.loginfo(f"DEBUG: prober.sources = {prober.sources}")
            rospy.loginfo(f"DEBUG: prober.scene_ids = {prober.scene_ids}")
            
            # 추론 실행
            prober.run()

            if prober.log and prober.pretrained_ckpt_path in prober.log:
                results = prober.log[prober.pretrained_ckpt_path]
                if results:
                    response = results[-1]['response'].strip().lower()
                    
                    # 응답에서 질문 타입 추출
                    if 'numerical_question' in response:
                        return 'numerical_question'
                    elif 'waypoint_generation_question' in response:
                        return 'waypoint_generation_question'
                    elif 'object_reference_question' in response:
                        return 'object_reference_question'
                    else:
                        # 응답에 타입이 명확하지 않으면 fallback 사용
                        rospy.logwarn(f"LEO response unclear: {response}")
                        return self.fallback_classification(cleaned_question)
            
            # LEO 실패 시 fallback 사용
            return self.fallback_classification(cleaned_question)
        except Exception as e:
            rospy.logerr(f"Error in run_leo_inference: {e}")
            raise e

    def fallback_classification(self, question):
        question_lower = question.lower()
        
        # 숫자 관련 키워드
        numerical_keywords = ['how many', 'count', 'number', 'distance', 'far', 'close', 'meters', 'feet']
        # 객체 참조 키워드  
        object_keywords = ['what is', 'what color', 'what shape', 'describe', 'object', 'thing']
        # 웨이포인트 생성 키워드
        waypoint_keywords = ['go to', 'move to', 'navigate', 'path', 'route', 'direction']
        
        if any(keyword in question_lower for keyword in numerical_keywords):
            return 'numerical_question'
        elif any(keyword in question_lower for keyword in object_keywords):
            return 'object_reference_question'
        elif any(keyword in question_lower for keyword in waypoint_keywords):
            return 'waypoint_generation_question'
        else:
            return 'object_reference_question'  # 기본값
    
    def parse_leo_result(self, result):
        if 'numerical' in result.lower() or 'count' in result.lower():
            return 'numerical_question'
        elif 'waypoint' in result.lower() or 'navigation' in result.lower():
            return 'waypoint_generation_question'
        else:
            return 'object_reference_question'
    def run(self):
        print("Awaiting question...")
        while not rospy.is_shutdown():
            try:
                # 사용자 입력 받기
                user_input = input("> ")
                # 입력 정리 (surrogate 유니코드 문자만 제거)
                cleaned_input = user_input.encode('utf-8', errors='ignore').decode('utf-8').strip()
                if cleaned_input:
                    # 질문 타입 분류
                    question_type = self.classify_question_type(cleaned_input)
                    
                    # 결과 publish
                    question_msg = std_msgs.msg.String()
                    question_msg.data = json.dumps({
                        'question': cleaned_input,
                        'type': question_type,
                        'timestamp': rospy.Time.now().to_sec()
                    })
                    
                    self.question_type_pub.publish(question_msg)
                    rospy.loginfo(f"Cleaned input : {cleaned_input}")
                    rospy.loginfo(f"Question classified as: {question_type}")
                    print(f"Question type: {question_type}")
                    print("Awaiting next question...")
                
            except KeyboardInterrupt:
                rospy.loginfo("Keyboard interrupt received, shutting down...")
                break
            except Exception as e:
                rospy.logerr(f"Error in interaction loop: {e}")
                print("Error occurred. Please try again.")

if __name__ == "__main__":
    try:
        interaction_manager = InteractionManager()
        interaction_manager.run()
    except rospy.ROSInterruptException:
        pass