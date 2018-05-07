//  
//  Properties.c  
//  ��д�����ļ�  
//  
  
#include "shiki.h"  
  
#define KEY_SIZE        128 // key��������С  
#define VALUE_SIZE      128 // value��������С  
  
#define LINE_BUF_SIZE   256 // ��ȡ�����ļ���ÿһ�еĻ�������С  
  
typedef struct Properties {  
    char *key;  
    char *value;  
    struct Properties *pNext;  
}Properties;  
  
typedef struct PROPS_HANDLE {  
    Properties *pHead;  // ��������ͷ�ڵ�  
    char *filepath;     // �����ļ�·��  
}PROPS_HANDLE;  
  
static int createPropsNode(Properties **props);         // ����һ���ڵ�  
static int trimeSpace(const char *src,char *dest);      // ȥ�ո�  
static int saveConfig(const char *filepath,Properties *head);   // ���޸Ļ򱣴���������浽�ļ�  
  
// ��ʼ���������ɹ�����0��ʧ�ܷ��ط�0ֵ  
int init(const char *filepath,void **handle)  
{  
    int ret = 0;  
    FILE *fp = NULL;  
    Properties *pHead = NULL,*pCurrent = NULL, *pMalloc = NULL;  
    PROPS_HANDLE *ph = NULL;  
    char line[LINE_BUF_SIZE];               // ��Ŷ�ȡÿһ�еĻ�����  
    char keybuff[KEY_SIZE] = { 0 };         // ���key�Ļ�����  
    char valuebuff[VALUE_SIZE] = { 0 };     // ���value�Ļ�����  
    char *pLine = NULL;                     // ÿ�л��������ݵ�ָ��  
      
    if(filepath == NULL || handle == NULL)  
    {  
        ret = -1;  
        printf("fun init error:%d from (filepath == NULL || handler == NULL)\n",ret);  
        return ret;  
    }  
      
    ph = (PROPS_HANDLE *)malloc(sizeof(PROPS_HANDLE));  
    if (ph == NULL) {  
        ret = -2;  
        printf("fun init malloc handle error:%d",ret);  
        return ret;  
    }  
    memset(ph, 0, sizeof(PROPS_HANDLE));  
      
    // ���ļ�  
    fp = fopen(filepath, "r");  
    if (!fp) {  
        ret = -3;  
        printf("fun init open file error:%d from %s\n",ret,filepath);  
        return ret;  
    }  
      
    // ����ͷ�ڵ�  
    ret = createPropsNode(&pHead);  
    if (ret != 0) {  
        fclose(fp);  // �ر��ļ�  
        printf("fun init create head node error:%d\n",ret);  
        return ret;  
    }  
    memset(pHead, 0, sizeof(Properties));  
      
    // ��������ͷ�ڵ���ļ�·����handle��  
    ph->pHead = pHead;  
    ph->filepath = (char *)malloc(strlen(filepath) + 1);  
    strcpy(ph->filepath, filepath);  
      
    pCurrent = pHead;  
  
    // ��ȡ�����ļ��е���������  
    while (!feof(fp)) {  
        if(fgets(line, LINE_BUF_SIZE, fp) == NULL)  
        {  
            break;  
        }  
          
        // �ҵȺ�  
        if ((pLine = strstr(line, "=")) == NULL) {   // û�еȺţ�������ȡ��һ��  
            continue;  
        }  
          
        // ѭ�������ڵ�  
        ret = createPropsNode(&pMalloc);  
        if (ret != 0) {  
            fclose(fp);  // �ر��ļ�  
            release((void **)&ph);  // �����ڵ�ʧ�ܣ��ͷ�������Դ  
            printf("create new node error:%d\n",ret);  
            return ret;  
        }  
  
        // ����Key  
        memcpy(keybuff, line, pLine-line);  
        trimeSpace(keybuff, pMalloc->key);    // ��keybuffȥ�ո��ŵ�pMallock.key��  
      
        // ����Value  
        pLine += 1;  
        trimeSpace(pLine, valuebuff);  
        strcpy(pMalloc->value, valuebuff);  
          
        // ���½ڵ�������  
        pMalloc->pNext = NULL;  
        pCurrent->pNext = pMalloc;  
        pCurrent = pMalloc; // ��ǰ�ڵ�����  
          
        // ����key,value  
        memset(keybuff, 0, KEY_SIZE);  
        memset(valuebuff, 0, VALUE_SIZE);  
    }  
      
    // ���û��������������  
    *handle = ph;  
      
    // �ر��ļ�  
    fclose(fp);  
      
    return ret;  
}  
  
// ��ȡ�����������ɹ�����0��ʧ�ܷ��ط�0ֵ  
int getCount(void *handle, int *count)  
{  
    int ret = 0,cn = 0;  
    PROPS_HANDLE *ph = NULL;  
    Properties *pCurrent = NULL;  
    if (handle == NULL || count == NULL) {  
        ret = -1;  
        printf("fun getCount error:%d from (handle == NULL || count == NULL)\n",ret);  
        return ret;  
    }  
    ph = (PROPS_HANDLE *)handle;  
    pCurrent = ph->pHead->pNext;  
    while (pCurrent != NULL) {  
        cn++;  
        pCurrent = pCurrent->pNext;  
    }  
      
    *count = cn;  
      
    return ret;  
}  
  
// ����KEY��ȡֵ���ҵ�����0�����δ�ҵ����ط�0ֵ  
int getValue(void *handle, const char *key, char *value)  
{  
    int ret = 0;  
    PROPS_HANDLE *ph = NULL;  
    Properties *pCurrent = NULL;  
    if (handle == NULL || key == NULL || value == NULL) {  
        ret = -1;  
        printf("getValue error:%d from (handle == NULL || key == NULL || value == NULL)\n",ret);  
        return ret;  
    }  
      
    ph = (PROPS_HANDLE *)handle;  
    pCurrent = ph->pHead->pNext;  
    while (pCurrent != NULL) {  
        if (strcmp(pCurrent->key,key) == 0) {  
            break;  
        }  
        pCurrent = pCurrent->pNext;  
    }  
      
    if (pCurrent == NULL) {  
        ret = -2;  
        printf("fun getValue warning: not found the key:%s\n",key);  
        return ret;  
    }  
      
    strcpy(value, pCurrent->value);  
      
    return ret;  
}  
  
// �޸�key��Ӧ������ֵ���޸ĳɹ�����0��ʧ�ܷ��ط�0ֵ  
int setValue(void *handle, const char *key, const char *value)  
{  
    int ret = 0;  
    PROPS_HANDLE *ph = NULL;  
    Properties *pCurrent = NULL;  
    if (handle == NULL || key == NULL || value == NULL) {  
        ret = -1;  
        printf("fun setValue error:%d from (handle == NULL || key == NULL || value == NULL)\n",ret);  
        return ret;  
    }  
      
    // ��û������  
    ph = (PROPS_HANDLE *)handle;  
      
    // �ӻ�������л�ȡͷ�ڵ�  
    pCurrent = ph->pHead->pNext;  
    while (pCurrent != NULL) {  
        if (strcmp(pCurrent->key, key) == 0) {  // �ҵ�  
            break;  
        }  
        pCurrent = pCurrent->pNext;  
    }  
      
    if (pCurrent == NULL) { // δ�ҵ�key  
        ret = -2;  
        printf("fun setValue warning: not found the key:%s\n",key);  
        return ret;  
    }  
      
    // �޸�key��value  
    strcpy(pCurrent->value, value);  
    if (strchr(value, '\n') == NULL) {  // ��һ�����з�  
        strcat(pCurrent->value, "\n");  
    }  
      
    // ���޸ĵ�������д�뵽�ļ�  
    ret = saveConfig(ph->filepath, ph->pHead);  
    
    return ret;  
}  
  
// ���һ�����ԣ���ӳɹ�����0��ʧ�ܷ��ط�0ֵ  
int add(void *handle, const char *key, const char *value)  
{  
    int ret = 0;  
    PROPS_HANDLE *ph = NULL;  
    Properties *pCurrent = NULL;  
    if (handle == NULL || key == NULL || value == NULL) {  
        ret = -1;  
        printf("fun add error:%d from (handle == NULL || key == NULL || value == NULL)\n",ret);  
        return ret;  
    }  
      
    ph = (PROPS_HANDLE *)handle;  
      
    //-----------���key���������У���ֱ���޸ģ�������ӵ�������-----------//  
    pCurrent = ph->pHead;  
    while (pCurrent->pNext != NULL) {  
        if (strcmp(pCurrent->pNext->key,key) == 0) {  
            break;  
        }  
        pCurrent = pCurrent->pNext;  
    }  
      
    if (pCurrent->pNext != NULL) {  
        return setValue(handle, key, value);  
    }  
      
    //-----------key�����ڣ�����һ���µ��������ӵ�������-----------//  
    Properties *pMalloc;  
    ret = createPropsNode(&pMalloc);  
    if (ret != 0) {  
        printf("fun add error:%d from malloc new node.",ret);  
        return ret;  
    }  
      
    strcpy(pMalloc->key, key);  
    if (strchr(pCurrent->value,'\n') == NULL) {  
        strcat(pCurrent->value, "\n");  
    }  
    strcpy(pMalloc->value, value);  
    if (strchr(value, '\n') == NULL) {  // ��һ�����з�  
        strcat(pMalloc->value, "\n");  
    }  
    pCurrent->pNext = pMalloc;  // ��������������  
      
    // ����������д�뵽�ļ�  
    ret = saveConfig(ph->filepath, ph->pHead);  
      
    return ret;  
}  
  
// ɾ��һ�����ԣ�ɾ���ɹ�����0��ʧ�ܷ��ط�0ֵ  
int del(void *handle, const char *key)  
{  
    int ret = 0;  
    PROPS_HANDLE *ph = NULL;  
    Properties *pCurrent = NULL, *pPrev = NULL;  
    if (handle == NULL || key == NULL) {  
        ret = -1;  
        printf("fun del error:%d from (handle == NULL || key == NULL)\n",ret);  
        return ret;  
    }  
      
    ph = (PROPS_HANDLE *)handle;  
    pPrev = ph->pHead;  
    pCurrent = ph->pHead->pNext;  
      
    while (pCurrent != NULL) {  
        if (strcmp(pCurrent->key, key) == 0) {  
            break;  
        }  
        pPrev = pCurrent;           // ��һ���ڵ�����  
        pCurrent = pCurrent->pNext; // ��ǰ�ڵ�����  
    }  
      
    if (pCurrent == NULL) { // û���ҵ�  
        ret = -2;  
        printf("fun del warning:not found the key:%s\n",key);  
        return  ret;  
    }  
      
    pPrev->pNext = pCurrent->pNext; // ��������ɾ��  
    free(pCurrent); // �ͷ��ڴ�  
    pCurrent = NULL;  
      
    // ���浽�ļ�  
    ret = saveConfig(ph->filepath, ph->pHead);  
      
    return ret;  
}  
  
// ��ȡ�����ļ������е�key����ȡ�ɹ�����0��ʧ�ܷ��ط�0ֵ  
int getKeys(void *handle, char ***keys, int *keyscount)  
{  
    int ret = 0, count = 0, index = 0;  
    PROPS_HANDLE *ph = NULL;  
    Properties *pCurrent = NULL;  
    char **pKeys = NULL;  
    if (handle == NULL || keys == NULL || keyscount == NULL) {  
        ret = -1;  
        printf("fun getKeys error:%d from (handle == NULL || keys == NULL || keyscount == NULL) \n",ret);  
        return ret;  
    }  
      
    // ��ȡ����������  
    ret = getCount(handle, &count);  
    if (ret != 0) {  
        printf("fun getKeys error:%d from getCount \n",ret);  
        return ret;  
    }  
      
    ph = (PROPS_HANDLE *)handle;  
    pCurrent = ph->pHead->pNext;  
      
    // ���������ȣ������ڴ�ռ�  
    pKeys = (char **)malloc(sizeof(char *) * count);  
    if (pKeys == NULL) {  
        ret = -2;  
        printf("fun getKeys error:%d from malloc keys\n",ret);  
        return ret;  
    }  
      
    pCurrent = ph->pHead->pNext;  
    while (pCurrent != NULL) {  
        pKeys[index] = pCurrent->key;  
        pCurrent = pCurrent->pNext;  
        index++;  
    }  
      
    *keys = pKeys;  
    *keyscount = count;  
      
    return ret;  
}  
  
// �ͷ�����key���ڴ�ռ䣬�ɹ�����0��ʧ�ܷ��ط�0ֵ  
int free_keys(char ***keys,int *keyscount)  
{  
    int ret = 0;  
    if (keys == NULL || keyscount == NULL) {  
        ret = -1;  
        printf("fun free_keys error:%d from (keys == NULL || keyscount == NULL) \n",ret);  
        return ret;  
    }  
      
    free(*keys);  
    *keys = NULL;  
    *keyscount = 0;  
      
    return ret;  
}  
  
// ��ȡ�����ļ������е�ֵ���ɹ�����0��ʧ�ܷ��ط�0ֵ  
int getValues(void *handle, char ***values, int *valuescount)  
{  
    int ret = 0, count = 0, index = 0;  
    PROPS_HANDLE *ph = NULL;  
    Properties *pCurrent = NULL;  
    char **pValues = NULL;  
    if (handle == NULL || values == NULL || valuescount == NULL) {  
        ret = -1;  
        printf("fun getValues error:%d from (handle == NULL || values == NULL || valuescount == NULL)\n",ret);  
        return ret;  
    }  
      
    // ��ȡ����������  
    ret = getCount(handle, &count);  
    if (ret != 0) {  
        printf("fun getValues error:%d from getCount \n",ret);  
        return ret;  
    }  
      
    // �����ڴ�ռ䣬������е�value  
    pValues = (char **)malloc(sizeof(char *) * count);  
    if (pValues == NULL) {  
        ret = -2;  
        printf("fun getValues error:%d from malloc values\n",ret);  
        return ret;  
    }  
      
    ph = (PROPS_HANDLE *)handle;  
    pCurrent = ph->pHead->pNext;  
    while (pCurrent != NULL) {  
        pValues[index] = pCurrent->value;  
        pCurrent = pCurrent->pNext;  
        index++;  
    }  
      
    *values = pValues;  
    *valuescount = count;  
      
    return ret;  
}  
  
// �ͷ�����value���ڴ�ռ䣬�ɹ�����0��ʧ�ܷ��ط�0ֵ  
int free_values(char ***values, int *valuescount)  
{  
    int ret = 0;  
    if (values == NULL || valuescount == NULL) {  
        ret = -1;  
        printf("fun free_values error:%d from (values == NULL || valuescount == NULL) \n",ret);  
        return ret;  
    }  
      
    free(*values);  
    *values = NULL;  
    *valuescount = 0;  
      
    return ret;  
}  
  
// �ͷŻ�����Դ���ɹ�����0��ʧ�ܷ��ط�0ֵ  
int release(void **handle)  
{  
    int ret = 0;  
    PROPS_HANDLE *ph = NULL;  
    if(handle == NULL)  
    {  
        ret = -1;  
        printf("release error:%d from (handler == NULL)\n",ret);  
        return ret;  
    }  
      
    ph = (PROPS_HANDLE *)*handle;  
      
    // �ͷ������ڴ���Դ  
    Properties *pCurr = ph->pHead;  
    Properties *pTemp = NULL;  
      
    while (pCurr != NULL) {  
        if (pCurr->key != NULL) {  
            free(pCurr->key);  
            pCurr->key = NULL;  
        }  
          
        if (pCurr->value != NULL) {  
            free(pCurr->value);  
            pCurr->value = NULL;  
        }  
          
        pTemp = pCurr->pNext;  
          
        free(pCurr);  
          
        pCurr = pTemp;  
    }  
      
    // �ͷŴ�������ļ�·��������ڴ�ռ�  
    if(ph->filepath != NULL)  
    {  
        free(ph->filepath);  
        ph->filepath = NULL;  
    }  
      
    // �ͷŻ����������  
    free(ph);  
    *handle = NULL;    // ����Ұָ��  
          
    return ret;  
}  
  
// ȥ�ո�  
static int trimeSpace(const char *src,char *dest)  
{  
    int ret = 0;  
    if (src == NULL || dest == NULL) {  
        ret = -1;  
        printf("trimeSpace error:%d from (src == NULL || dest == NULL)\n",ret);  
        return ret;  
    }  
      
    const char *psrc = src;  
    unsigned long i = 0,j = strlen(psrc) - 1,len;  
    while (psrc[i] == ' ')  
    {  
        i++;  
    }  
      
    while (psrc[j] == ' ') {  
        j--;  
    }  
      
    len = j - i + 1;  
      
    memcpy(dest,psrc+i,len);  
    *(dest+len) = '\0';  
      
    return ret;  
}  
  
// ����һ���ڵ�  
static int createPropsNode(Properties **props)  
{  
    int ret = 0;  
    Properties *p = NULL;  
    if (props == NULL) {  
        ret = -100;  
        printf("createProps error:%d from (props == NULL)\n",ret);  
        return ret;  
    }  
      
    p = (Properties *)malloc(sizeof(Properties));  
    if (p == NULL) {  
        ret = -200;  
        printf("createProps malloc %ld bytes error:%d\n",sizeof(Properties),ret);  
        return ret;  
    }  
    p->key = (char *)malloc(KEY_SIZE);  
    p->value = (char *)malloc(VALUE_SIZE);  
    p->pNext = NULL;  
      
    *props = p;  
      
    return ret;  
}  
  
// ���浽�ļ�  
static int saveConfig(const char *filepath,Properties *head)  
{  
    int ret = 0,writeLen = 0;  
    FILE *fp = NULL;  
    Properties *pCurrent = NULL;  
    if (filepath == NULL || head == NULL) {  
        ret = -100;  
        printf("fun saveConfig error:%d from (filepath == NULL || head == NULL)\n",ret);  
        return ret;  
    }  
      
    fp = fopen(filepath,"w");  
    if (fp == NULL) {  
        ret = -200;  
        printf("fun saveConfig:open file error:%d from %s\n",ret,filepath);  
        return ret;  
    }  
      
    pCurrent = head->pNext;  
    while (pCurrent != NULL) {  
        writeLen = fprintf(fp, "%s=%s",pCurrent->key,pCurrent->value);    // ����д����ֽ��������ִ��󷵻�һ����ֵ  
        if (writeLen < 0) {  //TODO ���д��ʧ�ܣ���ν�д������ݻ��ˣ�����  
            ret = -300;  
            printf("fun saveConfig err:%d from (%s=%s)\n",ret,pCurrent->key,pCurrent->value);  
            break;  
        }  
        pCurrent = pCurrent->pNext;  
    }  
  
    fclose(fp); // �ر��ļ�  
      
    return ret;  
}  