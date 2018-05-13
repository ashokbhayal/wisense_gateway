#include <thingspeak.h>
#include <ts_http.h>


ssize_t ts_http_post(ts_context_t *ctx, char *host_p, char *page_p, char *poststr_p)
{
    int sockfd;
    struct sockaddr_in servaddr;
    char sendline[MAXLINE + 1], recvline[MAXLINE + 1];
    ssize_t n;
    struct hostent *hptr;
    char hstr[50];
    

    /*
     * host_p is string "api.thingspeak.com"
     * page_p is "/update"
     * poststr_p is for example "field1=25" 
     */ 

    if ((hptr = gethostbyname(host_p)) == NULL) {
        fprintf(stderr, "gethostbyname error for host_p: %s: %s",
            host_p, hstrerror(h_errno));
        return -1;
    }

    if ((hptr->h_addrtype == AF_INET) && (hptr->h_addr_list) != NULL) {
        inet_ntop(hptr->h_addrtype, hptr->h_addr_list[0], hstr, sizeof(hstr));
    } 
    else
    {
        fprintf(stderr, "Error call inet_ntop \n");
    }

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(80);
    inet_pton(AF_INET, hstr, &servaddr.sin_addr);
    connect(sockfd, (SA *) & servaddr, sizeof(servaddr));

    
    snprintf(sendline, MAXSUB,
         "POST %s HTTP/1.1\r\n"
         "Host: %s\r\n"
         "Connection: close\r\n"
         "X-THINGSPEAKAPIKEY: %s\r\n"
         "Content-Type: application/x-www-form-urlencoded\r\n"
         "Content-Length: %lu\r\n\r\n"
         "%s",
         page_p, 
         host_p,
         ctx->api_key,
         (long unsigned int)strlen(poststr_p),
         poststr_p);

#if TS_DEBUG    
    printf("%s\n\n\n", sendline);
#endif

    write(sockfd, sendline, strlen(sendline));

    while ((n = read(sockfd, recvline, MAXLINE)) > 0)
    {
        recvline[n] = '\0';
#if TS_DEBUG        
        printf("%s", recvline);
#endif
    }

    close(sockfd);

    return n;
}



char *ts_http_get(char *host_p, char *page_p, char *buffer)
{
    int sockfd;
    FILE *sockwrap = NULL;
    struct sockaddr_in servaddr;
    /*char buffer[MAXLINE];*/
    ssize_t n;
    struct hostent *hptr;
    char hstr[50];
    /*char *http_ans = NULL;*/
    int headers = 1;
    


    if ((hptr = gethostbyname(host_p)) == NULL) {
        fprintf(stderr, "gethostbyname error for host_p: %s: %s", host_p, hstrerror(h_errno));
        return NULL;
    }

#if TS_DEBUG    
    printf("host_pname: %s\n", hptr->h_name);
#endif

    if ((hptr->h_addrtype == AF_INET) && (hptr->h_addr_list) != NULL)
        inet_ntop(hptr->h_addrtype, hptr->h_addr_list[0], hstr, sizeof(hstr));
    else
        fprintf(stderr, "Error call inet_ntop \n");

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(80);
    inet_pton(AF_INET, hstr, &servaddr.sin_addr);
    connect(sockfd, (SA *) & servaddr, sizeof(servaddr));

    
    sprintf(buffer, "GET %s HTTP/1.0\r\n"
                     "Host: %s\r\n\r\n", 
                     page_p, host_p);

    n = strlen(buffer);
    
    if(send(sockfd, buffer, strlen(buffer), 0) != n)
            fprintf(stderr, "Error call send()\n");

    bzero(buffer, MAXLINE);

    sockwrap = fdopen(sockfd, "r");

    if(fgets(buffer, MEM_SIZE(buffer), sockwrap) == NULL)
    {
            if (ferror(sockwrap))
                   fprintf(stderr, "Error sockwrap\n");
            else
            fprintf(stderr, "No response: timeout\n");
    }

    if(strstr(buffer, "200") == NULL)
    {
        /* If fails, print the line */
        fprintf(stderr, "%s\n", buffer);
        return NULL;
    }

    /*Skip over headers by going to single blank line*/

    while(headers)
    {
        if (fgets(buffer, MEM_SIZE(buffer), sockwrap) == NULL)
        {
            if(ferror(sockwrap))
                fprintf(stderr, "Error sockwrap\n");
            else
                fprintf(stderr, "No response: timeout\n");
        }

        if(strcmp("\r\n", buffer) == 0)
            headers = 0;
    }

    
    fread(buffer, strlen(buffer), MEM_SIZE(buffer), sockwrap);
    fclose(sockwrap);
    close(sockfd);

    return buffer;
}
