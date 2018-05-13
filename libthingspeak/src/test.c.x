#include <thingspeak.h>
#include <ts_http.h>


char TS_channelAPI[ ] = "3VU9R2UH8GA6SX38";
ts_feed_id_t feedId = 0;

int main(int argc, char **argv)
{
    ts_context_t *ctx_p = NULL;
    ts_datapoint_t data;
    char *result_p = NULL;

    if (argc < 2)
	return 1;

    ctx_p = ts_create_context(TS_channelAPI, 0);
    result_p = (char*)malloc(sizeof(char)*MAXLINE);
    if (result_p == NULL)
    {
        printf("malloc() failed !! \n");
	return 2;
    }
    bzero(result_p, MAXLINE);

    ts_set_value_i32(&data, atoi(argv[1]));
    
    ts_datastream_update(ctx_p, 0, "field1", &data);

    printf("ts_datastream_update() called ... \n");

    printf("\n\nJSON:\n%s\n", ts_feed_get_all(ctx_p, 0, TS_DATA_JSON, result_p));
    printf("\n\nCSV:\n%s\n", ts_datastream_get(ctx_p, 0, TS_DATA_CSV, "field2", result_p));

    ts_delete_context(ctx_p);

    printf("exiting ... \n");

    return 0;
}
