# KOBA (Keyboardless Object Bringing Assistant)

Tiago GPT-4 project for HRI course

[DEMO VIDEO](https://www.youtube.com/watch?v=W3SLLaTTGQ0)

## ASR and GPT-4

Speech recognition uses [Microsoft Azure speech-to-text](https://learn.microsoft.com/en-us/azure/cognitive-services/speech-service/how-to-recognize-speech?pivots=programming-language-python) and is working all the time. For the conversation logic and decision-making processes, we use the [GPT-4 API](https://platform.openai.com/docs/api-reference/completions) (8,192 tokens). Text intended for speech synthesis is always sent to Tiago following punctuation marks. The marker id is then shared on the `recognized` topic.

Before using install requirements:
```
cd tiago_gpt
pip install -r requirements.txt
```




