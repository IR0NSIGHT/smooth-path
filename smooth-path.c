/*
 *      smooth-path.c - Smooth Path plugin for the GIMP
 *      
 *      Copyright 2009 Marko Peric
 *      
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *      
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *      
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *      MA 02110-1301, USA.
 */

#include <libgimp/gimp.h>
#include <libgimp/gimpui.h>
#include <math.h>

#define rad_to_deg(angle) ((angle) * 360.0 / (2.0 * G_PI))

#define PLUG_IN_PROC "plug-in-smooth-path"
#define PLUG_IN_BINARY "smooth-path"
#define SCALE_WIDTH 125

static void query(void);
static void run(const gchar      *name,
                gint              nparams,
                const GimpParam  *param,
                gint             *nreturn_vals,
                GimpParam       **return_vals);

GimpPlugInInfo PLUG_IN_INFO =
{
    NULL,
    NULL,
    query,
    run
};

typedef struct
{
    gint32   smooth_specified;
    gdouble  ang_min;
    gdouble  ang_max;
} SmoothVals;

static SmoothVals svals =
{
    FALSE,
     60.0,
    120.0
};


MAIN()

/*----------------------------------------------------------------------------- 
 *  query  --  called so that the plugin can inform the GIMP about itself 
 *-----------------------------------------------------------------------------
 */
static void query (void)
{
    static GimpParamDef args[] =
    {
        {GIMP_PDB_INT32,    "run-mode",  "Interactive, non-interactive"},
        {GIMP_PDB_IMAGE,    "image",     "Input image"},
        {GIMP_PDB_VECTORS,  "path",      "Input path"},
        {GIMP_PDB_INT32,    "smooth",    "Smooth specified corners"},
        {GIMP_PDB_FLOAT,    "angle_min", "Minimum angle to be smoothed"},
        {GIMP_PDB_FLOAT,    "angle_max", "Maximum angle to be smoothed"},
    };

    gimp_install_procedure(
        PLUG_IN_PROC,
        "Smooth a path using Bezier interpolation",
        "An alternate name for this algorithm is cubic spline interpolation",
        "Marko Peric",
        "Marko Peric",
        "October 2009",
        "Smooth Path...",
        "*",
        GIMP_PLUGIN,
        G_N_ELEMENTS(args), 0,
        args, NULL);

    gimp_plugin_menu_register("plug-in-smooth-path", "<Vectors>");
}

/*----------------------------------------------------------------------------- 
 *  angle_between  --  determines the abs(angle) between two vectors formed by
 *                     the points va, vb, vc (i.e. va-->vb, vb-->vc) in degrees
 *-----------------------------------------------------------------------------
 */
gboolean angle_between(gdouble vax, gdouble vay, gdouble vbx, gdouble vby,
                       gdouble vcx, gdouble vcy)
{
    gdouble v1x, v1y, v2x, v2y, ret;
    /* Ugly I know, but laziness comes before beauty */
    if (!svals.smooth_specified) return -1;
    v1x = vbx - vax;
    v1y = vby - vay;
    v2x = vcx - vbx;
    v2y = vcy - vby;
    ret = 180 - ABS(rad_to_deg(atan2(-v1y*v2x + v1x*v2y, v1x*v2x + v1y*v2y)));
    if (svals.ang_max > svals.ang_min)
        return (ret < svals.ang_max && ret > svals.ang_min);
    else
        return (ret < svals.ang_max || ret > svals.ang_min);
}

/*----------------------------------------------------------------------------- 
 *  triagonal_solve  --  solves a tridiagonal system of simultaneous equations
 *-----------------------------------------------------------------------------
 */
void triagonal_solve(GArray *asb, GArray *asd)
{
    gdouble x = 1.0;
    gdouble id;
    gint i;
    GArray *asc;
    asc = g_array_new(FALSE, FALSE, sizeof(gdouble));
    for (i = 0; i < asb->len; i++)
        g_array_append_val(asc, x);
    g_array_index(asc, gdouble, 0) *= 0.25;
    g_array_index(asd, gdouble, 0) *= 0.25;
    for (i = 1; i < asb->len; i++) {
        id = 4.0 - g_array_index(asc, gdouble, i - 1);
        g_array_index(asc, gdouble, i) /= id;
        g_array_index(asd, gdouble, i) = (g_array_index(asd, gdouble, i) - 
                                      g_array_index(asd, gdouble, i - 1)) / id;
    }
    g_array_index(asb, gdouble, asb->len - 1) = g_array_index(asd, gdouble, 
                                                           asb->len - 1);
    for (i = asb->len - 2; i >= 0; i--)
        g_array_index(asb, gdouble, i) = g_array_index(asd, gdouble, i) -
                                      g_array_index(asc, gdouble, i) *
                                      g_array_index(asb, gdouble, i + 1);
    g_array_free(asc, TRUE);
}

/*----------------------------------------------------------------------------- 
 *  set_bezier_path  --  starting from a set of control points in a GIMP stroke
 *                       generate a new set of control points, and hence a new
 *                       path such that the Bezier curves are smoothly
 *                       interpolated between each other
 *-----------------------------------------------------------------------------
 */
void set_bezier_path(gint32 new_vectors_id, gint32 vectors_id, gint stroke_id)
{
    gboolean closed;
    gdouble *ctlpts;
    GArray *asd;
    GArray *asb;
    GArray *acx;
    GArray *acy;
    GArray *aconx1;
    GArray *aconx2;
    GArray *acony1;
    GArray *acony2;
    gint n, num_points, len;
    gdouble hx, hy;
    
    gimp_vectors_stroke_get_points(vectors_id, stroke_id, &num_points,
                                   &ctlpts, &closed);
                                   
    /* Must have at least 3 anchor points, i.e. 18 array entries */
    if (num_points < 18) {
        gimp_vectors_stroke_new_from_points(new_vectors_id, 
                                            GIMP_VECTORS_STROKE_TYPE_BEZIER,
                                            num_points, ctlpts, closed);
        g_free(ctlpts);
        return;
    }
    /* Look how many arrays we create, very wasteful! */
    asd = g_array_new(FALSE, TRUE, sizeof(gdouble));
    asb = g_array_new(FALSE, TRUE, sizeof(gdouble));
    acx = g_array_new(FALSE, TRUE, sizeof(gdouble));
    acy = g_array_new(FALSE, TRUE, sizeof(gdouble));
    aconx1 = g_array_new(FALSE, TRUE, sizeof(gdouble));
    aconx2 = g_array_new(FALSE, TRUE, sizeof(gdouble));
    acony1 = g_array_new(FALSE, TRUE, sizeof(gdouble));
    acony2 = g_array_new(FALSE, TRUE, sizeof(gdouble));
    
    /* Initialise anchor point array */
    len = num_points / 6;
    g_array_set_size(acx, len);
    g_array_set_size(acy, len);
    for (n = 0; n < acx->len; n++) {
        g_array_index(acx, gdouble, n) = ctlpts[n * 6 + 2];
        g_array_index(acy, gdouble, n) = ctlpts[n * 6 + 3];
    }
    
    /* Prepend last point, and append first two points if closed */
    if (closed) {
        g_array_prepend_val(acx, ctlpts[num_points - 4]);
        g_array_prepend_val(acy, ctlpts[num_points - 3]);
        g_array_append_val(acx, ctlpts[2]);
        g_array_append_val(acy, ctlpts[3]);
        g_array_append_val(acx, ctlpts[8]);
        g_array_append_val(acy, ctlpts[9]);
    }
    
    /* First for x */
    g_array_set_size(asd, acx->len - 2);
    g_array_set_size(asb, acx->len - 2);
    if (asd->len == 1) {
        g_array_index(asb, gdouble, 0) = 1.50 * g_array_index(acx, gdouble, 1) 
                                    - 0.25 * g_array_index(acx, gdouble, 0) 
                                    - 0.25 * g_array_index(acx, gdouble, 2);
    } else {
        g_array_index(asd, gdouble, 0) = 6 * g_array_index(acx, gdouble, 1) - 
                                       g_array_index(acx, gdouble, 0);
        for (n = 1; n < acx->len - 3; n++)
            g_array_index(asd, gdouble, n) = 6 * g_array_index(acx, gdouble, 
                                                               n + 1);
        g_array_index(asd, gdouble, acx->len - 3) 
                = 6 * g_array_index(acx, gdouble, acx->len - 2) - 
                  g_array_index(acx, gdouble, acx->len - 1);
        triagonal_solve(asb, asd);
    }
    g_array_prepend_val(asb, g_array_index(acx, gdouble, 0));
    g_array_append_val(asb, g_array_index(acx, gdouble, acx->len - 1));
    
    /* These will hold the interior control points, for x */
    for (n = 1; n < acx->len; n++) {
        hx = 2 * g_array_index(asb, gdouble, n - 1) / 3 +
             g_array_index(asb, gdouble, n) / 3;
        g_array_append_val(aconx1, hx);
        hx = g_array_index(asb, gdouble, n - 1) / 3 +
             2 * g_array_index(asb, gdouble, n) / 3;
        g_array_append_val(aconx2, hx);
    }
    
    /* Now for y */
    g_array_set_size(asd, acy->len - 2);
    g_array_set_size(asb, acy->len - 2);
    if (asd->len == 1) {
        g_array_index(asb, gdouble, 0) = 1.50 * g_array_index(acy, gdouble, 1) 
                                    - 0.25 * g_array_index(acy, gdouble, 0) 
                                    - 0.25 * g_array_index(acy, gdouble, 2);
    } else {
        g_array_index(asd, gdouble, 0) = 6 * g_array_index(acy, gdouble, 1) - 
                                       g_array_index(acy, gdouble, 0);
        for (n = 1; n < acy->len - 3; n++)
            g_array_index(asd, gdouble, n) = 6 * g_array_index(acy, gdouble, 
                                                               n + 1);
        g_array_index(asd, gdouble, acy->len - 3) 
                = 6 * g_array_index(acy, gdouble, acy->len - 2) - 
                  g_array_index(acy, gdouble, acy->len - 1);
        triagonal_solve(asb, asd);
    }
    g_array_prepend_val(asb, g_array_index(acy, gdouble, 0));
    g_array_append_val(asb, g_array_index(acy, gdouble, acy->len - 1));
    
    /* These will hold the interior control points, for y */
    for (n = 1; n < acy->len; n++) {
        hy = 2 * g_array_index(asb, gdouble, n - 1) / 3 +
             g_array_index(asb, gdouble, n) / 3;
        g_array_append_val(acony1, hy);
        hy = g_array_index(asb, gdouble, n - 1) / 3 +
             2 * g_array_index(asb, gdouble, n) / 3;
        g_array_append_val(acony2, hy);
    }
    
    /* Remove first two and last two control points if closed */
    if (closed) {
        g_array_remove_index(aconx1, aconx1->len - 1);
        g_array_remove_index(aconx1, 0);
        g_array_remove_index(aconx2, aconx2->len - 1);
        g_array_remove_index(aconx2, 0);
        g_array_remove_index(acony1, acony1->len - 1);
        g_array_remove_index(acony1, 0);
        g_array_remove_index(acony2, acony2->len - 1);
        g_array_remove_index(acony2, 0);
    }
    
    /* Now update our original ctlpts array */
    /* First two points are last aconx,y2 if closed, otherwise stay the same */
    if (closed) {
        if (angle_between(ctlpts[num_points-4], ctlpts[num_points-3], 
                          ctlpts[2], ctlpts[3], ctlpts[8], ctlpts[9])) {
            ctlpts[0] = g_array_index(aconx2, gdouble, aconx2->len - 1);
            ctlpts[1] = g_array_index(acony2, gdouble, acony2->len - 1);
        }
    }
    /* The interior points */
    for (n = 0; n < len - 1; n++) {
        if (n == 0) {
            if (closed && angle_between(ctlpts[num_points-4], 
                                        ctlpts[num_points-3], 
                                        ctlpts[2], ctlpts[3],
                                        ctlpts[8], ctlpts[9])) {
                ctlpts[n * 6 + 4] = g_array_index(aconx1, gdouble, n);
                ctlpts[n * 6 + 5] = g_array_index(acony1, gdouble, n);
            } else if (!closed && !svals.smooth_specified) {
                ctlpts[n * 6 + 4] = g_array_index(aconx1, gdouble, n);
                ctlpts[n * 6 + 5] = g_array_index(acony1, gdouble, n);
            }
        } else {
            if (angle_between(ctlpts[n * 6 - 4], ctlpts[n * 6 - 3], 
                              ctlpts[n * 6 + 2], ctlpts[n * 6 + 3], 
                              ctlpts[n * 6 + 8], ctlpts[n * 6 + 9])) {
                ctlpts[n * 6 + 4] = g_array_index(aconx1, gdouble, n);
                ctlpts[n * 6 + 5] = g_array_index(acony1, gdouble, n);
            }
        }
        if (n == len - 2) {
            if (closed && angle_between(ctlpts[num_points-10], 
                                        ctlpts[num_points-9], 
                                        ctlpts[num_points-4], 
                                        ctlpts[num_points-3], 
                                        ctlpts[2], ctlpts[3])) {
                ctlpts[n * 6 + 6] = g_array_index(aconx2, gdouble, n);
                ctlpts[n * 6 + 7] = g_array_index(acony2, gdouble, n);
            } else if (!closed && !svals.smooth_specified) {
                ctlpts[n * 6 + 6] = g_array_index(aconx2, gdouble, n);
                ctlpts[n * 6 + 7] = g_array_index(acony2, gdouble, n);
            }
        } else {
            if (angle_between(ctlpts[n * 6 + 2], ctlpts[n * 6 + 3],
                              ctlpts[n * 6 + 8], ctlpts[n * 6 + 9],
                              ctlpts[n * 6 + 14], ctlpts[n * 6 + 15])) {
                ctlpts[n * 6 + 6] = g_array_index(aconx2, gdouble, n);
                ctlpts[n * 6 + 7] = g_array_index(acony2, gdouble, n);
            }
        }
    }
    /* Last two points are last aconx,y1 if closed, otherwise stay the same */
    if (closed) {
        if (angle_between(ctlpts[num_points-10], ctlpts[num_points-9], 
                          ctlpts[num_points-4], ctlpts[num_points-3], 
                          ctlpts[2], ctlpts[3])) {
            ctlpts[num_points-2] = g_array_index(aconx1, gdouble, 
                                                 aconx1->len - 1);
            ctlpts[num_points-1] = g_array_index(acony1, gdouble, 
                                                 acony1->len - 1);
        }
    }
    
    /* Create a new stroke based on new ctlpts */
    gimp_vectors_stroke_new_from_points(new_vectors_id, 
                                        GIMP_VECTORS_STROKE_TYPE_BEZIER,
                                        num_points, ctlpts, closed);
    
    g_array_free(asd, TRUE);
    g_array_free(asb, TRUE);
    g_array_free(acx, TRUE);
    g_array_free(acy, TRUE);
    g_array_free(aconx1, TRUE);
    g_array_free(aconx2, TRUE);
    g_array_free(acony1, TRUE);
    g_array_free(acony2, TRUE);
    g_free(ctlpts);
}

/*----------------------------------------------------------------------------- 
 *  smooth_path  --  manipulates the vectors with Bezier smoothing algorithm
 *-----------------------------------------------------------------------------
 */
gboolean smooth_path(gint32 image_id, gint32 vectors_id)
{
    gint32  new_vectors_id;
    gint    n, num_strokes;
    gint   *strokes;
    gchar  *v_name;
    
    /* We create a new vector and delete the old one (undo doesn't
     * work if you simply change the strokes of an existing vector) */
    strokes = gimp_vectors_get_strokes(vectors_id, &num_strokes);
    v_name = gimp_vectors_get_name(vectors_id);
    new_vectors_id = gimp_vectors_new(image_id, v_name);
    
    /* The bezier smoothing algorithm is applied to each stroke */
    for (n = 0; n < num_strokes; n++) 
      set_bezier_path(new_vectors_id, vectors_id, strokes[n]);
      
    gimp_image_add_vectors(image_id, new_vectors_id, 
                           gimp_image_get_vectors_position(image_id, 
                                                           vectors_id));
    gimp_image_remove_vectors(image_id, vectors_id);
    gimp_vectors_set_name(new_vectors_id, v_name);
    g_free(v_name);
    g_free(strokes);
    
    return TRUE;
}

/*----------------------------------------------------------------------------- 
 *  smooth_dialog  --  dialog that allows user to set some algorithm parameters
 *-----------------------------------------------------------------------------
 */
gboolean smooth_dialog(void)
{
    GtkWidget *dialog;
    GtkWidget *vbox;
    GtkWidget *toggle;
    GtkWidget *table;
    GtkObject *scale1_data;
    GtkObject *scale2_data;
    gboolean   run;
    
    gimp_ui_init (PLUG_IN_BINARY, FALSE);
    
    dialog = gimp_dialog_new("Smooth Path", PLUG_IN_BINARY,
                             NULL, 0,
                             gimp_standard_help_func, PLUG_IN_PROC,
                             GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
                             GTK_STOCK_OK,     GTK_RESPONSE_OK,
                             NULL);
                             
    gtk_dialog_set_alternative_button_order(GTK_DIALOG (dialog),
                                            GTK_RESPONSE_OK,
                                            GTK_RESPONSE_CANCEL,
                                            -1);
                                            
    gtk_window_set_resizable (GTK_WINDOW (dialog), FALSE);
    
    vbox = gtk_vbox_new(FALSE, 12);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 12);
    gtk_container_add(GTK_CONTAINER(GTK_DIALOG(dialog)->vbox), vbox);
    gtk_widget_show(vbox);
    
    toggle 
      = gtk_check_button_new_with_mnemonic("_Smooth only specified corners");
    gtk_box_pack_start (GTK_BOX (vbox), toggle, FALSE, FALSE, 0);
    gtk_widget_show(toggle);
    g_signal_connect(toggle, "toggled",
                     G_CALLBACK(gimp_toggle_button_update),
                     &svals.smooth_specified);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(toggle),
                                 (svals.smooth_specified == TRUE));
                     
    table = gtk_table_new(2, 3, FALSE);
    gtk_table_set_col_spacings(GTK_TABLE(table), 6);
    gtk_table_set_row_spacings(GTK_TABLE(table), 6);
    gtk_table_set_row_spacing(GTK_TABLE(table), 0, 4);
    gtk_box_pack_start(GTK_BOX(vbox), table, FALSE, FALSE, 0);
    gtk_widget_show(table);
    
    scale1_data = gimp_scale_entry_new(GTK_TABLE(table), 0, 0,
                                       "Mi_nimum angle:", SCALE_WIDTH, 6,
                                       svals.ang_min, 0.0, 180.0, 1.0, 15.0, 2,
                                       TRUE, 0, 0, NULL, NULL);
    g_signal_connect(scale1_data, "value-changed",
                     G_CALLBACK(gimp_double_adjustment_update),
                     &svals.ang_min);
                     
    scale2_data = gimp_scale_entry_new(GTK_TABLE(table), 0, 1,
                                       "Ma_ximum angle:", SCALE_WIDTH, 6,
                                       svals.ang_max, 0.0, 180.0, 1.0, 15.0, 2,
                                       TRUE, 0, 0, NULL, NULL);
    g_signal_connect(scale2_data, "value-changed",
                     G_CALLBACK(gimp_double_adjustment_update),
                     &svals.ang_max);
                     
    gtk_widget_show(dialog);
    
    run = (gimp_dialog_run(GIMP_DIALOG(dialog)) == GTK_RESPONSE_OK);

    gtk_widget_destroy(dialog);

    return run;
}

/*----------------------------------------------------------------------------- 
 *  run  --  code that gets called when the plugin is asked to run
 *-----------------------------------------------------------------------------
 */
static void run(const gchar      *name,
                gint              nparams,
                const GimpParam  *param,
                gint             *nreturn_vals,
                GimpParam       **return_vals)
{
    static GimpParam  values[1];
    GimpPDBStatusType status = GIMP_PDB_SUCCESS;
    GimpRunMode       run_mode;
    gint32            image_id, vectors_id; 

    /* Setting mandatory output values */
    *nreturn_vals = 1;
    *return_vals  = values;
    values[0].type = GIMP_PDB_STATUS;
    values[0].data.d_status = status;
    
    run_mode = param[0].data.d_int32;
    image_id = param[1].data.d_image;
    vectors_id = param[2].data.d_int32;
    
    switch (run_mode) {
        case GIMP_RUN_INTERACTIVE:
            /* Get options last values if needed */
            gimp_get_data(PLUG_IN_PROC, &svals);
            /* Display the dialog */
            if (!smooth_dialog())
                return;
            break;
        case GIMP_RUN_NONINTERACTIVE:
            if (nparams != 6)
                status = GIMP_PDB_CALLING_ERROR;
            if (status == GIMP_PDB_SUCCESS) {
                svals.smooth_specified = param[3].data.d_int32;
                svals.ang_min = param[4].data.d_float;
                svals.ang_max = param[5].data.d_float;
            }
            break;
        case GIMP_RUN_WITH_LAST_VALS:
            /*  Get options last values if needed  */
            status = GIMP_PDB_PASS_THROUGH;
            break;
        default:
            break;
    }
    
    if (status == GIMP_PDB_SUCCESS) {
        /* Bundle the smooth_path code inside an undo group */        
        gimp_image_undo_group_start(image_id);
        smooth_path(image_id, vectors_id);
        gimp_image_undo_group_end(image_id);
      
        /* Refresh and clean up */
        if (run_mode != GIMP_RUN_NONINTERACTIVE)
            gimp_displays_flush();

        /*  Finally, set options in the core  */
        if (run_mode == GIMP_RUN_INTERACTIVE)
              gimp_set_data(PLUG_IN_PROC, &svals, sizeof(SmoothVals));
    }
    
    values[0].data.d_status = status;
}
