/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * This is the stream state machine for audio source.
 */

#include "wiced_bt_sdp.h"
#include "wiced_bt_a2dp_src_int.h"

/*****************************************************************************
** Constants and types
*****************************************************************************/

/* State table for init state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_source_state_evt_t enum*/
static const wiced_bt_a2dp_source_event_map_t wiced_bt_a2dp_source_sst_init[] =
{
    {WICED_BT_A2DP_SOURCE_API_CONNECT_EVT          , WICED_BT_A2DP_SOURCE_SIG_OPENING_SST, wiced_bt_a2dp_source_do_sdp},
    {WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT       , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_cleanup},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT         , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_hdl_str_close_ind},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT         , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_hdl_str_close_cfm},
    {WICED_BT_A2DP_SOURCE_AVDT_CONNECT_EVT         , WICED_BT_A2DP_SOURCE_SIG_OPEN_SST   , wiced_bt_a2dp_source_do_sdp},
    {WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_sig_closed_cleanup}
};

/* State table for signal opening state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_source_state_evt_t enum*/
static const wiced_bt_a2dp_source_event_map_t wiced_bt_a2dp_source_sst_sig_opening[] =
{
    {WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT       , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_cleanup},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT          , WICED_BT_A2DP_SOURCE_SIG_OPEN_SST   , wiced_bt_a2dp_source_connect_req},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT        , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_sdp_failed},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT        , WICED_BT_A2DP_SOURCE_SIG_OPENING_SST, wiced_bt_a2dp_source_hdl_str_close_ind},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT        , WICED_BT_A2DP_SOURCE_SIG_OPENING_SST, wiced_bt_a2dp_source_hdl_str_close_cfm},
    {WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_sig_open_fail}
};

/* State table for signaling channel opened state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_source_state_evt_t enum*/
static const wiced_bt_a2dp_source_event_map_t wiced_bt_a2dp_source_sst_sig_open[] =
{
    {WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT       , WICED_BT_A2DP_SOURCE_SIG_OPEN_SST   , wiced_bt_a2dp_source_sig_hdl_ap_close_disconnect_req},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT          , WICED_BT_A2DP_SOURCE_SIG_OPEN_SST,    wiced_bt_a2dp_source_sig_opened},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT        , WICED_BT_A2DP_SOURCE_SIG_OPEN_SST   , wiced_bt_a2dp_source_sdp_failed},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT         , WICED_BT_A2DP_SOURCE_SIG_OPENING_SST, wiced_bt_a2dp_source_hdl_str_close_ind},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT         , WICED_BT_A2DP_SOURCE_SIG_OPENING_SST, wiced_bt_a2dp_source_hdl_str_close_cfm},
    {WICED_BT_A2DP_SOURCE_AVDT_CONNECT_EVT         , WICED_BT_A2DP_SOURCE_SIG_OPEN_SST   , wiced_bt_a2dp_source_sig_opened},
    {WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_sig_closed_cleanup},
    {WICED_BT_A2DP_SOURCE_API_AVDT_SETCONFIG       , WICED_BT_A2DP_SOURCE_OUTGOING_SST   , wiced_bt_a2dp_source_sig_hdl_setconfig},
    {WICED_BT_A2DP_SOURCE_STR_DISCOVER_CFM_EVT    , WICED_BT_A2DP_SOURCE_SIG_OPEN_SST   , wiced_bt_a2dp_source_sig_str_hdl_discovery_cfm},
    {WICED_BT_A2DP_SOURCE_STR_GETCAP_CFM_EVT       ,WICED_BT_A2DP_SOURCE_SIG_OPEN_SST   , wiced_bt_a2dp_source_sig_str_hdl_getcap_cfm},
    {WICED_BT_A2DP_SOURCE_STR_CONFIG_IND_EVT       , WICED_BT_A2DP_SOURCE_INCOMING_SST   , wiced_bt_a2dp_source_sig_str_hdl_setconfig}
};

/* State table for incoming state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_source_state_evt_t enum*/
static const wiced_bt_a2dp_source_event_map_t wiced_bt_a2dp_source_sst_incoming[] =
{
    {WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT       , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_disconnect_req},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT          , WICED_BT_A2DP_SOURCE_INCOMING_SST   , wiced_bt_a2dp_source_free_sdb},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT        , WICED_BT_A2DP_SOURCE_INCOMING_SST   , wiced_bt_a2dp_source_free_sdb},
    {WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT          , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_str_opened},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT        , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_str_open_fail},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT        , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_str_open_fail},
    {WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_sig_closed_cleanup}
};

/* State table for outgoing state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_source_state_evt_t enum*/
static const wiced_bt_a2dp_source_event_map_t wiced_bt_a2dp_source_sst_outgoing[] =
{
    {WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT       , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_disconnect_req},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT          , WICED_BT_A2DP_SOURCE_OUTGOING_SST   , wiced_bt_a2dp_source_free_sdb},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT        , WICED_BT_A2DP_SOURCE_OUTGOING_SST   , wiced_bt_a2dp_source_free_sdb},
    {WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT          , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_str_opened},
    {WICED_BT_A2DP_SOURCE_STR_OPEN_FAIL_EVT        , WICED_BT_A2DP_SOURCE_INIT_SST     , wiced_bt_a2dp_source_str_open_fail},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT        , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_str_open_fail},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT        , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_str_open_fail},
    {WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_sig_closed_cleanup}
};

/* State table for open state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_source_state_evt_t enum*/
static const wiced_bt_a2dp_source_event_map_t wiced_bt_a2dp_source_sst_open[] =
{
    {WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT       , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_do_close},
    {WICED_BT_A2DP_SOURCE_API_START_EVT            , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_do_start},
    {WICED_BT_A2DP_SOURCE_API_START_RESP_EVT       , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_send_start_resp},
    {WICED_BT_A2DP_SOURCE_API_SUSPEND_EVT          , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_str_stopped},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT          , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_free_sdb},
    {WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT        , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_free_sdb},
    {WICED_BT_A2DP_SOURCE_STR_START_IND_EVT        , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_start_ind},
    {WICED_BT_A2DP_SOURCE_STR_START_CFM_EVT        , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_start_ok},
    {WICED_BT_A2DP_SOURCE_STR_START_FAIL_EVT       , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_start_failed},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT        , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_hdl_str_close_ind},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT        , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_hdl_str_close_cfm},
    {WICED_BT_A2DP_SOURCE_STR_SUSPEND_CFM_EVT      , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_suspend_cfm},
    {WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_sig_closed_cleanup},
    {WICED_BT_A2DP_SOURCE_API_RECONFIG_CMD_EVT     , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_sig_hdl_reconfig},
    {WICED_BT_A2DP_SOURCE_STR_RECONFIG_CFM_EVT     , WICED_BT_A2DP_SOURCE_OPEN_SST       , wiced_bt_a2dp_source_reconfig_cfm}
};

/* State table for closing state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_source_state_evt_t enum*/
static const wiced_bt_a2dp_source_event_map_t wiced_bt_a2dp_source_sst_closing[] =
{
    {WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT       , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_disconnect_req},
    {WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT          , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_do_close},
    {WICED_BT_A2DP_SOURCE_STR_OPEN_FAIL_EVT        , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_disconnect_req},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT         , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_hdl_str_close_ind},
    {WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT         , WICED_BT_A2DP_SOURCE_CLOSING_SST    , wiced_bt_a2dp_source_hdl_str_close_cfm},
    {WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SOURCE_INIT_SST       , wiced_bt_a2dp_source_sig_closed_cleanup},
    {WICED_BT_A2DP_SOURCE_STR_DISCOVER_CFM_EVT    , WICED_BT_A2DP_SOURCE_SIG_OPEN_SST   , wiced_bt_a2dp_source_sig_str_hdl_discovery_cfm},
};

/* State table */
static const wiced_bt_a2dp_source_sst_tbl_entry_t wiced_bt_a2dp_sst_tbl[] =
{
    {wiced_bt_a2dp_source_sst_init,       sizeof(wiced_bt_a2dp_source_sst_init)/sizeof(wiced_bt_a2dp_source_event_map_t)},
    {wiced_bt_a2dp_source_sst_sig_opening,sizeof(wiced_bt_a2dp_source_sst_sig_opening)/sizeof(wiced_bt_a2dp_source_event_map_t)},
    {wiced_bt_a2dp_source_sst_sig_open,   sizeof(wiced_bt_a2dp_source_sst_sig_open)/sizeof(wiced_bt_a2dp_source_event_map_t)},
    {wiced_bt_a2dp_source_sst_incoming,   sizeof(wiced_bt_a2dp_source_sst_incoming)/sizeof(wiced_bt_a2dp_source_event_map_t)},
    {wiced_bt_a2dp_source_sst_outgoing,   sizeof(wiced_bt_a2dp_source_sst_outgoing)/sizeof(wiced_bt_a2dp_source_event_map_t)},
    {wiced_bt_a2dp_source_sst_open,       sizeof(wiced_bt_a2dp_source_sst_open)/sizeof(wiced_bt_a2dp_source_event_map_t)},
    {wiced_bt_a2dp_source_sst_closing,    sizeof(wiced_bt_a2dp_source_sst_closing)/sizeof(wiced_bt_a2dp_source_event_map_t)},
};

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_search_event
**
** Description      Search for the event in the state table and return the
**                       next state and handler if the event is handled by the state table.
**
** Returns      Returns WICED_TRUE if the event is handled by the state table. Else return WICED_FALSE.
**
*******************************************************************************/
static wiced_bool_t wiced_bt_a2dp_source_search_event(
    wiced_bt_a2dp_source_sst_tbl_entry_t state_table_entry, uint8_t event,
    uint8_t *p_next_state, wiced_bt_a2dp_source_sm_act_t *pfhandler)
{
    int l = 0;
    int r = state_table_entry.size_of_table - 1;
    int m = 0;
    wiced_bool_t found = WICED_FALSE;
    const wiced_bt_a2dp_source_event_map_t *state_table = state_table_entry.state_table;

    while (l <= r)
    {
        m = l + (r-l)/2;
        if (state_table[m].event == event) /* Check if x is present at mid */
        {
            found = WICED_TRUE;
            break;
        }
        else if (state_table[m].event < event) /* If x greater, ignore left half */
        {
            l = m + 1;
        }
        else /* If x is smaller, ignore right half */
        {
            r = m - 1;
        }
    }

    if(found == WICED_FALSE)
    {
        return WICED_FALSE;
    }

    *p_next_state = state_table[m].next_state;
    *pfhandler = state_table[m].pfhandler;
    return WICED_TRUE;
}


/*******************************************************************************
**
** Function         wiced_bt_a2dp_csm_execute
**
** Description      Stream state machine event handling function for A2DP source
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_source_ssm_execute(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data, uint8_t event)
{
    wiced_bool_t                       found      = WICED_FALSE;
    wiced_bt_a2dp_source_sm_act_t        pfhandler  = NULL;
    wiced_bt_a2dp_source_sst_tbl_entry_t state_table_entry;
    uint8_t                            next_state;

    WICED_BTA2DP_SRC_TRACE("[wiced_bt_a2dp_source_ssm_execute] p_ccb state %d and event %d \r\n", p_ccb->state, event);
    if (p_ccb->p_scb == NULL)
    {
        /* This stream is not registered */
        WICED_BTA2DP_SRC_ERROR("%s: ERROR: channel not registered \r\n", __FUNCTION__);
        return;
    }

    /* Look up the state table for the current state */
    state_table_entry = wiced_bt_a2dp_sst_tbl[p_ccb->state];

    /* Search whether this event is handled by the current state */
    found = wiced_bt_a2dp_source_search_event(state_table_entry, event, &next_state, &pfhandler);
    if(found == WICED_FALSE)
    {
        WICED_BTA2DP_SRC_TRACE("%s: event=0x%x state=0x%x not handled \r\n", __FUNCTION__,
            event, p_ccb->state);
        return;
    }

#if (defined(WICED_BT_A2DP_SOURCE_DEBUG) && WICED_BT_A2DP_SOURCE_DEBUG == TRUE)
    WICED_BTA2DP_SRC_TRACE("%s: current-state=%s event=%s next-state=%s \r\n",
            __FUNCTION__,
            wiced_bt_a2dp_source_st_code(p_ccb->state),
            wiced_bt_a2dp_source_evt_code(event),
            wiced_bt_a2dp_source_st_code(next_state));
#else
    WICED_BTA2DP_SRC_TRACE("%s: current-state=0x%x event=0x%x next-state=0x%x \r\n",
            __FUNCTION__, p_ccb->state, event, next_state);
#endif

    p_ccb->state  = next_state;

    if(pfhandler != NULL)
    {
        WICED_BTA2DP_SRC_TRACE(" ---> Entering the function handler ! \r\n");
        pfhandler(p_ccb, p_data);
        WICED_BTA2DP_SRC_TRACE(" <--- Exiting the function handler ! \r\n");
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_init_state_machine
**
** Description    Initialize state machine.
**
** Returns          WICED_SUCCESS if successfully initialized, else return WICED_FAILURE.
**
*******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_init_state_machine(void)
{
    /* Loop through each state and ensure that the events are listed in order */
    int num_states = sizeof(wiced_bt_a2dp_sst_tbl)/sizeof(wiced_bt_a2dp_source_sst_tbl_entry_t);
    int i=0;

    for(i=0; i<num_states; i++) {
        wiced_bt_a2dp_source_sst_tbl_entry_t state_table_entry = wiced_bt_a2dp_sst_tbl[i];
        const wiced_bt_a2dp_source_event_map_t *state_table = state_table_entry.state_table;
        uint8_t size_of_table = state_table_entry.size_of_table;
        int j=0;

        for(j=1; j<size_of_table; j++)
        {
            /* If value of jth event is less than (j-1)th event, then the order is incorrect */
            if(state_table[j].event < state_table[j-1].event)
            {
                WICED_BTA2DP_SRC_ERROR("%s: Incorrect state table entry. state:%d, entry:%d. \
                    IMPORTANT: THE EVENTS MUST BE LISTED IN ASCENDING ORDER \r\n", __FUNCTION__,i,j);
                return WICED_BADARG;
            }
        }
    }
    return WICED_SUCCESS;
}


/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if (defined(WICED_BT_A2DP_SOURCE_DEBUG) && WICED_BT_A2DP_SOURCE_DEBUG == TRUE)
/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_st_code
**
** Description
**
** Returns          char *
**
*******************************************************************************/
char *wiced_bt_a2dp_source_st_code(uint8_t state)
{
    switch(state)
    {
        case WICED_BT_A2DP_SOURCE_INIT_SST:        return "INIT";
        case WICED_BT_A2DP_SOURCE_SIG_OPENING_SST: return "SIG_OPENING";
        case WICED_BT_A2DP_SOURCE_SIG_OPEN_SST:    return "SIG_OPEN";
        case WICED_BT_A2DP_SOURCE_INCOMING_SST:    return "INCOMING";
        case WICED_BT_A2DP_SOURCE_OUTGOING_SST:    return "Outgoing";
        case WICED_BT_A2DP_SOURCE_OPEN_SST:        return "OPEN";
        case WICED_BT_A2DP_SOURCE_CLOSING_SST:     return "CLOSING";
        default:                                 return "unknown";
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_evt_code
**
** Description
**
** Returns          char *
**
*******************************************************************************/
char *wiced_bt_a2dp_source_evt_code(uint8_t evt_code)
{
    switch(evt_code)
    {
        case WICED_BT_A2DP_SOURCE_INVALID_EVT:            return "INVALID";
        case WICED_BT_A2DP_SOURCE_API_DEINIT_EVT:         return "API_DEINIT";
        case WICED_BT_A2DP_SOURCE_SIG_CHG_EVT:            return "SIG_CHG";
        case WICED_BT_A2DP_SOURCE_AVDT_REPOPT_CONN_EVT:   return "AVDT_REPOPT_CONN";
        case WICED_BT_A2DP_SOURCE_API_CONNECT_EVT:        return "API_CONNECT";
        case WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT:     return "API_DISCONNECT";
        case WICED_BT_A2DP_SOURCE_API_START_EVT:          return "API_START";
        case WICED_BT_A2DP_SOURCE_API_SUSPEND_EVT:        return "API_SUSPEND";
        case WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT:        return "SDP_DISC_OK";
        case WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT:      return "SDP_DISC_FAIL";
        case WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT:        return "STR_OPEN_OK";
        case WICED_BT_A2DP_SOURCE_STR_OPEN_FAIL_EVT:      return "STR_OPEN_FAIL";
        case WICED_BT_A2DP_SOURCE_STR_START_IND_EVT:      return "STR_START_IND";
        case WICED_BT_A2DP_SOURCE_STR_START_CFM_EVT:      return "STR_START_CFM";
        case WICED_BT_A2DP_SOURCE_STR_START_FAIL_EVT:     return "STR_START_FAIL";
        case WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT:       return "STR_CLOSE_IND";
        case WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT:       return "STR_CLOSE_CFM";
        case WICED_BT_A2DP_SOURCE_STR_SUSPEND_CFM_EVT:    return "STR_SUSPEND_CFM";
        case WICED_BT_A2DP_SOURCE_AVDT_CONNECT_EVT:       return "AVDT_CONNECT";
        case WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT:    return "AVDT_DISCONNECT";
        case WICED_BT_A2DP_SOURCE_API_AVDT_SETCONFIG:     return "SET_CONFIG_API";
        case WICED_BT_A2DP_SOURCE_STR_DISCOVER_CFM_EVT:     return "DISCOVER EVENT";
        case WICED_BT_A2DP_SOURCE_STR_GETCAP_CFM_EVT:     return "GET CAP EVENT";
        case WICED_BT_A2DP_SOURCE_API_RECONFIG_CMD_EVT:    return "API_RECONFIG";
        case WICED_BT_A2DP_SOURCE_STR_RECONFIG_CFM_EVT:    return "STR_RECONFIG_CFM";
        case WICED_BT_A2DP_SOURCE_STR_CONFIG_IND_EVT:      return "STR_CONFIG_IND";
        default:             return "unknown";
    }
}

#endif
