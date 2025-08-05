#!/usr/bin/env python3
"""
LEO 모델 미리 다운로드 스크립트
컨테이너 빌드 시 실행되어 모델을 캐싱합니다.
"""

import os
import sys
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM

def download_leo_models():
    """LEO에서 사용하는 모델들을 미리 다운로드"""
    
    print("Downloading LEO models for container caching...")
    
    models_to_download = [
        # 메인 LLM 모델
        {
            "name": "lmsys/vicuna-7b-v1.5",
            "type": "AutoModelForCausalLM",
            "kwargs": {"torch_dtype": torch.float16, "device_map": "auto", "trust_remote_code": True}
        },
        # OpenCLIP 모델 (이미지 처리)
        {
            "name": "openai/clip-vit-base-patch32",
            "type": "CLIPModel",
            "kwargs": {}
        },
        # CLIP 토크나이저
        {
            "name": "openai/clip-vit-base-patch32",
            "type": "CLIPTokenizer",
            "kwargs": {}
        }
    ]
    
    try:
        for model_info in models_to_download:
            print(f"Downloading {model_info['type']}: {model_info['name']}")
            
            if model_info['type'] == 'AutoModelForCausalLM':
                from transformers import AutoTokenizer, AutoModelForCausalLM
                tokenizer = AutoTokenizer.from_pretrained(model_info['name'])
                model = AutoModelForCausalLM.from_pretrained(model_info['name'], **model_info['kwargs'])
            elif model_info['type'] == 'CLIPModel':
                from transformers import CLIPModel
                model = CLIPModel.from_pretrained(model_info['name'], **model_info['kwargs'])
            elif model_info['type'] == 'CLIPTokenizer':
                from transformers import CLIPTokenizer
                tokenizer = CLIPTokenizer.from_pretrained(model_info['name'], **model_info['kwargs'])
        
        print("✅ All LEO models downloaded successfully!")
        
    except Exception as e:
        print(f"❌ Error downloading models: {e}")
        sys.exit(1)

if __name__ == "__main__":
    download_leo_models() 