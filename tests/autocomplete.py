import openai
# print hello world
openai.api_key = 'dummy'
openai.api_base = 'http://10.0.0.250:5000/v1'
# openai.api_base = 'http://you.bytelogics.com:5000/v1'
i=1/2
result = openai.Completion.create(engine='codegen', prompt='def hello', max_tokens=16, temperature=0.1, stop=["\n\n"])
print(result)

