import os
import requests

proxy = 'http://127.0.0.1:7890'
# 设置 HTTP 和 HTTPS 代理
# os.environ['HTTP_PROXY'] = 'http://127.0.0.1:7890'
# os.environ['HTTPS_PROXY'] = 'http://127.0.0.1:7890'
proxies = {
    "http": "http://your_proxy:port",
    "https": "http://your_proxy:port",
}
# 测试请求
try:
    response = requests.get("https://huggingface.co/api/datasets")
    print(response.json())
except requests.exceptions.RequestException as e:
    print(f"Error: {e}")
